# UART Recipe JSON Protocol

## Overview
This protocol is for HMI to send recipe stage parameters to firmware over UART.

- Header: `ba+`
- Command: `recipe`
- Format: `ba+recipe={json}`
- Payload must include `recipe_no` and `stage_no`.
- Firmware stores accepted values into NVS (Flash).

## Recipe Index Mapping

- `0`: Brisket
- `1`: Spare Ribs
- `2`: Pulled Pork
- `3`: Chicken

Note: firmware supports future expansion up to max recipe slots.

## Stage Mapping

- `0`: Smoke Ignition
- `1`: Cooking
- `2`: Drying
- `3`: Holding

## UART Command Format

Single-line command ending with newline (`\n`):

```text
ba+recipe={"recipe_no":0,"stage_no":1,"data":{"cook_minutes":480,"oven_min":105,"oven_max":125}}
```

## JSON Schema

Two payload styles are supported.

### Style A: `data` object (recommended)

```json
{
  "recipe_no": 0,
  "stage_no": 1,
  "data": {
    "cook_minutes": 480,
    "oven_min": 105,
    "oven_max": 125
  }
}
```

### Style B: flat keys (compatible)

```json
{
  "recipe_no": 0,
  "stage_no": 1,
  "cook_minutes": 480,
  "oven_min": 105,
  "oven_max": 125
}
```

## Stage Field Keys

### Stage 0: Smoke Ignition (`stage_no=0`)

- `fuel_type` (0 Pellet, 1 Charcoal, 2 Woodchip, 3 Hybrid)
- `smoke_enable`
- `ignite_t1`
- `ignite_t2`
- `pump_condition`
- `pump_power`
- `reignite_t1`
- `reignite_t2`
- `pump_on_sec`
- `pump_off_sec`

### Stage 1: Cooking (`stage_no=1`)

- `cook_minutes`
- `oven_min`
- `oven_max`
- `oven_error_pct`
- `heater_on_sec`
- `heater_off_sec`
- `fan_on_sec`
- `fan_off_sec`
- `spray_time_sec`
- `spray_power`

### Stage 2: Drying (`stage_no=2`)

- `dry_minutes`
- `oven_min`
- `oven_max`
- `oven_error_pct`
- `heater_on_sec`
- `heater_off_sec`

### Stage 3: Holding (`stage_no=3`)

- `hold_minutes`
- `oven_min`
- `oven_max`
- `oven_error_pct`
- `heater_on_sec`
- `heater_off_sec`

## Additional UART Commands

- Select active recipe:

```text
ba+set=20,<recipe_no>,0
```

- Read recipe profile:

```text
ba+get=20,<recipe_no>,0
```

If `<recipe_no>` is invalid in `ba+get=20`, firmware reads current active recipe.

## Firmware Response

On success:

```text
[UART] RECIPE_JSON applied: recipe=<n> stage=<s>
```

On failure examples:

```text
[UART] ba+recipe JSON parse error: ...
[UART] Invalid recipe_no. Use 0-3
[UART] Invalid stage_no. Use 0~3
[UART] Failed to apply recipe JSON (validation failed)
```

## Validation Rules (summary)

- `recipe_no` must be valid slot index.
- `stage_no` must be 0~3.
- range/order checks apply (for example `oven_min < oven_max`, `ignite_t1 < ignite_t2`, etc.).
- invalid field value is rejected and not committed.

## Notes

- Values are persisted in NVS immediately after accepted updates.
- If updated recipe is currently active, temperature limits are re-applied immediately.
