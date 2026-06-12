// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/thermal.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/device.h>
// Only for for_each_thermal_cooling_device
#include "../thermal_core.h"

#define DRV_NAME			"qcom-battery-thermal-policy"
#define DEFAULT_SOURCE_PSY		"battery"
#define DEFAULT_START_DELAY_MS		0
#define DEFAULT_RETRY_MS		1000

struct qbtp_map {
	struct device_node *cdev_np;
	int cdev_idx;
	u32 min_state;
	u32 max_state;
	u32 weight;
};

struct qbtp_trip {
	u32 id;
	int temp_mc;
	int hyst_mc;
	bool active;
	enum thermal_trip_type type;
	int num_maps;
	struct qbtp_map *maps;
};

struct qbtp_cdev {
	struct device_node *np;
	struct thermal_cooling_device *cdev;
	unsigned long last_state;
	bool last_state_valid;
};

struct qbtp_drv {
	struct device *dev;
	struct mutex lock;

	struct delayed_work start_work;
	struct work_struct eval_work;
	struct notifier_block psy_nb;
	bool notifier_registered;

	const char *source_psy_name;
	u32 start_delay_ms;
	u32 retry_delay_ms;

	struct power_supply *psy;
	bool ready;

	int num_trips;
	struct qbtp_trip *trips;

	int num_cdevs;
	struct qbtp_cdev *cdevs;
	int cdev_cap;

	int last_temp_mc;
	int active_trip;
};

struct qbtp_find_cdev_ctx {
	const struct device_node *np;
	struct thermal_cooling_device *cdev;
};

static enum thermal_trip_type qbtp_trip_type_from_str(const char *s)
{
	if (!strcmp(s, "active"))
		return THERMAL_TRIP_ACTIVE;
	if (!strcmp(s, "hot"))
		return THERMAL_TRIP_HOT;
	if (!strcmp(s, "critical"))
		return THERMAL_TRIP_CRITICAL;

	return THERMAL_TRIP_PASSIVE;
}

static void qbtp_put_map_nodes(struct qbtp_drv *drv)
{
	int i, j;

	if (!drv->trips)
		return;

	for (i = 0; i < drv->num_trips; i++) {
		struct qbtp_trip *trip = &drv->trips[i];

		if (!trip->maps)
			continue;

		for (j = 0; j < trip->num_maps; j++) {
			if (trip->maps[j].cdev_np) {
				of_node_put(trip->maps[j].cdev_np);
				trip->maps[j].cdev_np = NULL;
			}
			trip->maps[j].cdev_idx = -1;
		}
	}
}

static void qbtp_put_psy(struct qbtp_drv *drv)
{
	if (drv->psy) {
		power_supply_put(drv->psy);
		drv->psy = NULL;
	}
}

static void qbtp_release_cdev_refs(struct qbtp_drv *drv)
{
	int i;

	for (i = 0; i < drv->num_cdevs; i++) {
		drv->cdevs[i].cdev = NULL;
		drv->cdevs[i].np = NULL;
		drv->cdevs[i].last_state = 0;
		drv->cdevs[i].last_state_valid = false;
	}

	drv->num_cdevs = 0;
}

static int qbtp_validate_map_against_cdev(struct qbtp_drv *drv,
					  struct thermal_cooling_device *cdev,
					  const struct qbtp_map *map)
{
	unsigned long cdev_max_state;
	int ret;

	if (map->min_state > map->max_state) {
		dev_err(drv->dev,
			"bad map for %pOF: min_state=%u > max_state=%u\n",
			map->cdev_np, map->min_state, map->max_state);
		return -EINVAL;
	}

	if (!cdev->ops || !cdev->ops->get_max_state ||
	    !cdev->ops->set_cur_state) {
		dev_err(drv->dev,
			"cdev %s missing get_max_state/set_cur_state ops\n",
			cdev->type);
		return -EINVAL;
	}

	ret = cdev->ops->get_max_state(cdev, &cdev_max_state);
	if (ret) {
		dev_err(drv->dev, "failed get_max_state for %s: %d\n",
			cdev->type, ret);
		return ret;
	}

	if (map->max_state > cdev_max_state) {
		dev_err(drv->dev,
			"map for %pOF exceeds max_state: cfg=%u cdev=%lu\n",
			map->cdev_np, map->max_state, cdev_max_state);
		return -EINVAL;
	}

	return 0;
}

static int qbtp_scan_num_trips(struct qbtp_drv *drv, struct device_node *np)
{
	struct device_node *child;
	u32 reg, max_reg = 0;
	bool found = false;
	int ret;

	for_each_available_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &reg);
		if (ret) {
			dev_err(drv->dev, "%pOF: missing reg\n", child);
			of_node_put(child);
			return ret;
		}

		if (!found || reg > max_reg)
			max_reg = reg;
		found = true;
	}

	if (!found)
		return -EINVAL;

	return max_reg + 1;
}

static int qbtp_count_total_maps(struct device_node *np)
{
	struct device_node *trip_np;
	int total = 0;

	for_each_available_child_of_node(np, trip_np)
		total += of_get_available_child_count(trip_np);

	return total;
}

static int qbtp_parse_map(struct qbtp_drv *drv, struct device_node *np,
			  struct qbtp_map *map)
{
	int ret;

	map->cdev_np = of_parse_phandle(np, "qcom,cooling-device", 0);
	if (!map->cdev_np) {
		dev_err(drv->dev, "%pOF: missing qcom,cooling-device\n", np);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "qcom,min-state", &map->min_state);
	if (ret) {
		dev_err(drv->dev, "%pOF: missing qcom,min-state\n", np);
		goto err_put_np;
	}

	ret = of_property_read_u32(np, "qcom,max-state", &map->max_state);
	if (ret) {
		dev_err(drv->dev, "%pOF: missing qcom,max-state\n", np);
		goto err_put_np;
	}

	ret = of_property_read_u32(np, "qcom,weight", &map->weight);
	if (ret)
		map->weight = THERMAL_WEIGHT_DEFAULT;

	if (map->min_state > map->max_state) {
		dev_err(drv->dev, "%pOF: invalid state range %u..%u\n",
			np, map->min_state, map->max_state);
		ret = -EINVAL;
		goto err_put_np;
	}

	map->cdev_idx = -1;
	return 0;

err_put_np:
	of_node_put(map->cdev_np);
	map->cdev_np = NULL;
	return ret;
}

static int qbtp_parse_trip_maps(struct qbtp_drv *drv, struct device_node *trip_np,
				struct qbtp_trip *trip)
{
	struct device_node *map_np;
	int count, idx = 0, ret;

	count = of_get_available_child_count(trip_np);
	if (!count) {
		dev_err(drv->dev, "%pOF: trip has no map children\n", trip_np);
		return -EINVAL;
	}

	trip->maps = devm_kcalloc(drv->dev, count, sizeof(*trip->maps),
				  GFP_KERNEL);
	if (!trip->maps)
		return -ENOMEM;

	trip->num_maps = count;

	for_each_available_child_of_node(trip_np, map_np) {
		ret = qbtp_parse_map(drv, map_np, &trip->maps[idx]);
		if (ret) {
			of_node_put(map_np);
			return ret;
		}
		idx++;
	}

	return 0;
}

static int qbtp_parse_trips(struct qbtp_drv *drv, struct device_node *np)
{
	struct device_node *trip_np;
	int ret;

	drv->num_trips = qbtp_scan_num_trips(drv, np);
	if (drv->num_trips <= 0)
		return -EINVAL;

	drv->trips = devm_kcalloc(drv->dev, drv->num_trips,
				  sizeof(*drv->trips), GFP_KERNEL);
	if (!drv->trips)
		return -ENOMEM;

	for_each_available_child_of_node(np, trip_np) {
		struct qbtp_trip *trip;
		const char *type = "passive";
		u32 reg, temp_mc, hyst_mc;

		ret = of_property_read_u32(trip_np, "reg", &reg);
		if (ret) {
			of_node_put(trip_np);
			return ret;
		}
		if (reg >= drv->num_trips) {
			dev_err(drv->dev, "%pOF: reg=%u out of range\n",
				trip_np, reg);
			of_node_put(trip_np);
			return -EINVAL;
		}

		trip = &drv->trips[reg];
		trip->id = reg;
		trip->active = false;

		ret = of_property_read_u32(trip_np, "temperature", &temp_mc);
		if (ret) {
			dev_err(drv->dev, "%pOF: missing temperature\n", trip_np);
			of_node_put(trip_np);
			return ret;
		}
		trip->temp_mc = (int)temp_mc;

		ret = of_property_read_u32(trip_np, "hysteresis", &hyst_mc);
		if (ret) {
			dev_err(drv->dev, "%pOF: missing hysteresis\n", trip_np);
			of_node_put(trip_np);
			return ret;
		}
		trip->hyst_mc = (int)hyst_mc;

		ret = of_property_read_string(trip_np, "type", &type);
		if (ret)
			type = "passive";

		trip->type = qbtp_trip_type_from_str(type);

		ret = qbtp_parse_trip_maps(drv, trip_np, trip);
		if (ret) {
			of_node_put(trip_np);
			return ret;
		}
	}

	for (ret = 0; ret < drv->num_trips; ret++) {
		if (!drv->trips[ret].maps) {
			dev_err(drv->dev, "missing trip@%d\n", ret);
			return -EINVAL;
		}
	}

	return 0;
}

static int qbtp_parse_dt(struct qbtp_drv *drv, struct device_node *np)
{
	int ret;

	ret = of_property_read_string(np, "qcom,source-psy",
				      &drv->source_psy_name);
	if (ret)
		drv->source_psy_name = DEFAULT_SOURCE_PSY;

	ret = of_property_read_u32(np, "qcom,start-delay-ms",
				   &drv->start_delay_ms);
	if (ret)
		drv->start_delay_ms = DEFAULT_START_DELAY_MS;

	ret = of_property_read_u32(np, "qcom,retry-delay-ms",
				   &drv->retry_delay_ms);
	if (ret)
		drv->retry_delay_ms = DEFAULT_RETRY_MS;

	ret = qbtp_parse_trips(drv, np);
	if (ret)
		return ret;

	drv->cdev_cap = qbtp_count_total_maps(np);
	if (drv->cdev_cap <= 0)
		return -EINVAL;

	drv->cdevs = devm_kcalloc(drv->dev, drv->cdev_cap, sizeof(*drv->cdevs),
				  GFP_KERNEL);
	if (!drv->cdevs)
		return -ENOMEM;

	drv->last_temp_mc = INT_MIN;
	drv->active_trip = -1;
	drv->ready = false;

	return 0;
}

static int qbtp_get_source_psy(struct qbtp_drv *drv)
{
	union power_supply_propval val;
	int ret;

	if (drv->psy)
		return 0;

	drv->psy = power_supply_get_by_name(drv->source_psy_name);
	if (!drv->psy)
		return -EPROBE_DEFER;

	ret = power_supply_get_property(drv->psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret) {
		dev_err(drv->dev,
			"power_supply '%s' has no readable TEMP property: %d\n",
			drv->source_psy_name, ret);
		qbtp_put_psy(drv);
		return ret;
	}

	dev_info(drv->dev, "using source power_supply '%s'\n",
		 drv->source_psy_name);
	return 0;
}

static int qbtp_match_cdev_by_np(struct thermal_cooling_device *cdev, void *data)
{
	struct qbtp_find_cdev_ctx *ctx = data;

	if (cdev->np == ctx->np) {
		ctx->cdev = cdev;
		return 1;
	}

	return 0;
}

static struct thermal_cooling_device *
qbtp_find_cdev_by_np(const struct device_node *np)
{
	struct qbtp_find_cdev_ctx ctx = {
		.np = np,
		.cdev = NULL,
	};

	for_each_thermal_cooling_device(qbtp_match_cdev_by_np, &ctx);
	return ctx.cdev;
}

static int qbtp_find_cdev_slot_by_np(struct qbtp_drv *drv, struct device_node *np)
{
	int i;

	for (i = 0; i < drv->num_cdevs; i++) {
		if (drv->cdevs[i].np == np)
			return i;
	}

	return -ENOENT;
}

static int qbtp_resolve_cdevs(struct qbtp_drv *drv)
{
	int i, j, ret;

	qbtp_release_cdev_refs(drv);

	for (i = 0; i < drv->num_trips; i++) {
		struct qbtp_trip *trip = &drv->trips[i];

		for (j = 0; j < trip->num_maps; j++) {
			struct qbtp_map *map = &trip->maps[j];
			struct thermal_cooling_device *cdev;
			int idx;

			idx = qbtp_find_cdev_slot_by_np(drv, map->cdev_np);
			if (idx >= 0) {
				map->cdev_idx = idx;
				continue;
			}

			if (drv->num_cdevs >= drv->cdev_cap)
				return -EINVAL;

			cdev = qbtp_find_cdev_by_np(map->cdev_np);
			if (!cdev) {
				dev_info(drv->dev,
					 "cooling device for %pOF not ready\n",
					 map->cdev_np);
				qbtp_release_cdev_refs(drv);
				return -EPROBE_DEFER;
			}

			ret = qbtp_validate_map_against_cdev(drv, cdev, map);
			if (ret) {
				qbtp_release_cdev_refs(drv);
				return ret;
			}

			idx = drv->num_cdevs++;
			drv->cdevs[idx].np = map->cdev_np;
			drv->cdevs[idx].cdev = cdev;
			drv->cdevs[idx].last_state = 0;
			drv->cdevs[idx].last_state_valid = false;
			map->cdev_idx = idx;

			dev_info(drv->dev, "resolved cdev %s for %pOF\n",
				 cdev->type, map->cdev_np);
		}
	}

	for (i = 0; i < drv->num_trips; i++) {
		struct qbtp_trip *trip = &drv->trips[i];

		for (j = 0; j < trip->num_maps; j++) {
			struct qbtp_map *map = &trip->maps[j];

			if (map->cdev_idx < 0 || map->cdev_idx >= drv->num_cdevs)
				return -EINVAL;

			ret = qbtp_validate_map_against_cdev(
				drv, drv->cdevs[map->cdev_idx].cdev, map);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int qbtp_read_temp_mc(struct qbtp_drv *drv, int *temp_mc)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property(drv->psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return ret;

	/* power_supply TEMP is tenths of a degree Celsius -> milli-Celsius */
	*temp_mc = val.intval * 100;
	return 0;
}

static int qbtp_compute_active_trip(struct qbtp_drv *drv, int temp_mc)
{
	int i, active = -1;

	for (i = 0; i < drv->num_trips; i++) {
		struct qbtp_trip *trip = &drv->trips[i];
		int clear_temp_mc = trip->temp_mc - trip->hyst_mc;

		if (temp_mc >= trip->temp_mc)
			trip->active = true;
		else if (temp_mc < clear_temp_mc)
			trip->active = false;

		if (trip->active)
			active = i;
	}

	return active;
}

static unsigned long qbtp_target_state_for_cdev(struct qbtp_drv *drv, int cdev_idx)
{
	int i, j;

	if (drv->active_trip < 0)
		return 0;

	for (i = drv->active_trip; i >= 0; i--) {
		struct qbtp_trip *trip = &drv->trips[i];

		for (j = 0; j < trip->num_maps; j++) {
			struct qbtp_map *map = &trip->maps[j];

			if (map->cdev_idx == cdev_idx)
				return map->max_state;
		}
	}

	return 0;
}

static int qbtp_set_cdev_state(struct qbtp_drv *drv, int cdev_idx,
			       unsigned long target)
{
	struct qbtp_cdev *qcdev = &drv->cdevs[cdev_idx];
	unsigned long max_state;
	int ret;

	if (!qcdev->cdev || !qcdev->cdev->ops ||
	    !qcdev->cdev->ops->get_max_state ||
	    !qcdev->cdev->ops->set_cur_state)
		return -EINVAL;

	ret = qcdev->cdev->ops->get_max_state(qcdev->cdev, &max_state);
	if (ret)
		return ret;

	if (target > max_state)
		target = max_state;

	if (qcdev->last_state_valid && qcdev->last_state == target)
		return 0;

	ret = qcdev->cdev->ops->set_cur_state(qcdev->cdev, target);
	if (ret) {
		dev_err(drv->dev, "set_cur_state failed for %s: state=%lu ret=%d\n",
			qcdev->cdev->type, target, ret);
		return ret;
	}

	qcdev->last_state = target;
	qcdev->last_state_valid = true;
	return 0;
}

static void qbtp_apply_policy_locked(struct qbtp_drv *drv)
{
	int i;

	for (i = 0; i < drv->num_cdevs; i++) {
		unsigned long target = qbtp_target_state_for_cdev(drv, i);

		qbtp_set_cdev_state(drv, i, target);
	}
}

static void qbtp_eval_locked(struct qbtp_drv *drv)
{
	int temp_mc, ret;

	if (!drv->ready || !drv->psy)
		return;

	ret = qbtp_read_temp_mc(drv, &temp_mc);
	if (ret) {
		dev_err(drv->dev, "failed reading battery TEMP: %d\n", ret);
		return;
	}

	drv->last_temp_mc = temp_mc;
	drv->active_trip = qbtp_compute_active_trip(drv, temp_mc);
	qbtp_apply_policy_locked(drv);
}

static void qbtp_eval_workfn(struct work_struct *work)
{
	struct qbtp_drv *drv = container_of(work, struct qbtp_drv, eval_work);

	mutex_lock(&drv->lock);
	qbtp_eval_locked(drv);
	mutex_unlock(&drv->lock);
}

static void qbtp_start_workfn(struct work_struct *work)
{
	struct qbtp_drv *drv = container_of(to_delayed_work(work),
					    struct qbtp_drv, start_work);
	int ret;

	mutex_lock(&drv->lock);

	ret = qbtp_get_source_psy(drv);
	if (ret)
		goto retry;

	ret = qbtp_resolve_cdevs(drv);
	if (ret)
		goto retry;

	drv->ready = true;
	qbtp_eval_locked(drv);
	mutex_unlock(&drv->lock);

	dev_info(drv->dev, "battery thermal policy active\n");
	return;

retry:
	drv->ready = false;
	mutex_unlock(&drv->lock);

	dev_info(drv->dev, "source psy/cdevs not ready (ret=%d), retry in %u ms\n",
		 ret, drv->retry_delay_ms);
	queue_delayed_work(system_wq, &drv->start_work,
			   msecs_to_jiffies(drv->retry_delay_ms));
}

static int qbtp_psy_notifier_cb(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct qbtp_drv *drv = container_of(nb, struct qbtp_drv, psy_nb);
	struct power_supply *psy = data;

	if (event != PSY_EVENT_PROP_CHANGED || !psy)
		return NOTIFY_DONE;

	if (!drv->psy) {
		queue_delayed_work(system_wq, &drv->start_work, 0);
		return NOTIFY_OK;
	}

	if (psy == drv->psy)
		schedule_work(&drv->eval_work);

	return NOTIFY_OK;
}

static int qbtp_probe(struct platform_device *pdev)
{
	struct qbtp_drv *drv;
	int ret;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->dev = &pdev->dev;
	mutex_init(&drv->lock);
	INIT_DELAYED_WORK(&drv->start_work, qbtp_start_workfn);
	INIT_WORK(&drv->eval_work, qbtp_eval_workfn);

	ret = qbtp_parse_dt(drv, pdev->dev.of_node);
	if (ret) {
		qbtp_put_map_nodes(drv);
		return ret;
	}

	drv->psy_nb.notifier_call = qbtp_psy_notifier_cb;
	ret = power_supply_reg_notifier(&drv->psy_nb);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to register power_supply notifier: %d\n", ret);
		qbtp_put_map_nodes(drv);
		return ret;
	}
	drv->notifier_registered = true;

	platform_set_drvdata(pdev, drv);

	queue_delayed_work(system_wq, &drv->start_work,
			   msecs_to_jiffies(drv->start_delay_ms));

	dev_info(&pdev->dev, "scheduled battery thermal policy startup\n");
	return 0;
}

static int qbtp_remove(struct platform_device *pdev)
{
	struct qbtp_drv *drv = platform_get_drvdata(pdev);
	int i;

	if (drv->notifier_registered) {
		power_supply_unreg_notifier(&drv->psy_nb);
		drv->notifier_registered = false;
	}

	cancel_delayed_work_sync(&drv->start_work);
	cancel_work_sync(&drv->eval_work);

	mutex_lock(&drv->lock);

	drv->ready = false;
	drv->active_trip = -1;

	for (i = 0; i < drv->num_cdevs; i++) {
		if (drv->cdevs[i].cdev &&
		    (!drv->cdevs[i].last_state_valid ||
		     drv->cdevs[i].last_state != 0))
			qbtp_set_cdev_state(drv, i, 0);
	}

	qbtp_put_psy(drv);
	qbtp_release_cdev_refs(drv);

	mutex_unlock(&drv->lock);

	qbtp_put_map_nodes(drv);
	return 0;
}

static const struct of_device_id qbtp_of_match[] = {
	{ .compatible = "qcom,battery-thermal-policy" },
	{ }
};
MODULE_DEVICE_TABLE(of, qbtp_of_match);

static struct platform_driver qbtp_driver = {
	.probe = qbtp_probe,
	.remove = qbtp_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = qbtp_of_match,
	},
};
module_platform_driver(qbtp_driver);

MODULE_DESCRIPTION("Qualcomm battery-backed thermal policy");
MODULE_LICENSE("GPL");
