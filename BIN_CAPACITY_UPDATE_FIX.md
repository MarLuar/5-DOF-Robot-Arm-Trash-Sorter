# Bin Capacity Update Fix - After Drop

## Problem

Bin monitoring data wasn't being updated reliably after each object was dropped into the bin.

## Root Cause

The bin capacity was being sent **immediately** after the pickup sequence completed, but:
1. Arduino needs time to process the dropped object
2. Ultrasonic sensors need time to stabilize
3. Capacity readings might be stale or inaccurate

## Solution

Separated the bin capacity update from the classification webhook and added a **2-second delay** after drop to allow Arduino to update its sensors.

### How It Works Now

```
┌─────────────────────────────────────────────────────┐
│ 1. Pickup Sequence Completes                        │
│    Object dropped into bin                           │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│ 2. Classification Webhook (IMMEDIATE)               │
│    Sends: classification, confidence, model_name    │
│    Log: "✓ Webhook sent: non-biodegradable (87%)"   │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│ 3. Wait 2 Seconds                                   │
│    Arduino updates ultrasonic sensors               │
│    New capacity readings stabilize                  │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│ 4. Bin Capacity Webhook (DELAYED)                   │
│    Reads fresh capacity from Arduino                │
│    Sends: bio=45%, nonbio=30%                       │
│    Log: "📊 Bin capacity UPDATED: BIO=45%, NON-BIO=30%" │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│ 5. Recapture Empty Grid (1s later)                  │
│    Updates camera reference                         │
│    Ready for next object                            │
└─────────────────────────────────────────────────────┘
```

## Code Changes

### 1. Separated Webhook Calls (Line ~3536)
```python
# Send classification webhook immediately
self._safe_after(0, lambda c=cell: self.send_webhook_after_pickup(c))

# Send bin capacity monitoring after short delay (allow Arduino to update sensors)
self._safe_after(2000, lambda c=cell: self.send_bin_capacity_after_drop(c))

# Recapture empty grid after successful pickup
self._safe_after(1000, self._recapture_empty_grid_after_pickup)
```

### 2. New Function: send_bin_capacity_after_drop (Line ~3109)
```python
def send_bin_capacity_after_drop(self, cell: str):
    """Send bin capacity to Laravel after object is dropped (with delay for Arduino to update)"""
    # Read fresh capacity from Arduino
    # Send both bin fill levels to Laravel
    # Log the result
```

### 3. Removed Duplicate Send
Removed `send_bin_capacity_webhook` call from `send_webhook_after_pickup` to prevent duplicate sends.

## What You'll See in Logs

### Timeline:
```
Pickup for A1_NON complete! (Total: 12.456s)
Object picked up successfully!
✓ Webhook sent: non-biodegradable (87%)  ← IMMEDIATE
📊 Reading bin capacity after drop...    ← 2s delay
📊 Bin capacity UPDATED: BIO=45%, NON-BIO=30%  ← FRESH DATA
Empty grid reference updated             ← 1s later
```

### Success Case:
```
✓ Webhook sent: non-biodegradable (87%)
📊 Bin capacity UPDATED: BIO=45%, NON-BIO=30%
```

### Arduino Not Connected:
```
✓ Webhook sent: non-biodegradable (87%)
⚠ Arduino not connected, skipping bin capacity update
```

### Arduino No Response:
```
✓ Webhook sent: non-biodegradable (87%)
📊 Reading bin capacity after drop...
⚠ Arduino didn't respond to capacity request
```

## Benefits

✅ **Fresh Data**: 2-second delay ensures Arduino sensors are updated  
✅ **Accurate Readings**: Capacity reflects actual post-drop state  
✅ **Separate Concerns**: Classification and capacity sent independently  
✅ **Better Logging**: Clear indication of what's being sent and when  
✅ **Error Handling**: Detailed logs if Arduino doesn't respond  

## Testing

### Test Case 1: Normal Flow
1. Place object in grid
2. Wait for auto-pickup and drop
3. **Check logs for:**
   ```
   ✓ Webhook sent: ...
   📊 Reading bin capacity after drop...
   📊 Bin capacity UPDATED: BIO=XX%, NON-BIO=XX%
   ```
4. **Verify:** Both webhooks received by Laravel

### Test Case 2: Check Timing
1. Watch the log timestamps
2. **Should see:**
   - Classification webhook: Immediate (0s)
   - Bin capacity: 2 seconds later
   - Empty grid recapture: 1 second after classification

### Test Case 3: Arduino Disconnected
1. Disconnect Arduino
2. Run pickup
3. **Should see:**
   ```
   ✓ Webhook sent: ...
   ⚠ Arduino not connected, skipping bin capacity update
   ```

## Summary

| Issue | Before | After |
|-------|--------|-------|
| **Bin Capacity Timing** | Immediate (stale data) | 2s delay (fresh data) ✅ |
| **Webhook Separation** | Combined | Independent ✅ |
| **Arduino Update Time** | No delay | 2s wait ✅ |
| **Logging** | Minimal | Detailed ✅ |
| **Reliability** | Sometimes inaccurate | Always fresh ✅ |

Bin capacity is now updated **reliably after every drop** with fresh sensor data! 🎉
