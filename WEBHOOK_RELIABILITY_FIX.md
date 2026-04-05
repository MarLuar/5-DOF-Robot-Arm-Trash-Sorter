# Webhook Reliability Fix

## Problem

Webhook sometimes doesn't send data to the Laravel website after pickup.

## Root Causes Found

### Issue 1: Cell Name Mismatch ❌

The webhook function looked up classification using the full cell name (e.g., `A1_NON`), but the classification was stored under the base cell name (`A1`):

```python
# BEFORE - Wrong lookup
class_name, confidence = self.classification_results.get(cell, ('unknown', 0.0))
# cell = 'A1_NON' but classification stored under 'A1'
# Result: Returns ('unknown', 0.0) → Webhook SKIPPED ❌
```

### Issue 2: Silent Fail ❌

When webhook failed, there was no detailed logging to help debug:
- No indication of which cells had classification data
- No fallback mechanism
- Stack traces not shown

## Solution

### Fix 1: Strip _NON Suffix ✅

```python
# AFTER - Correct lookup
base_cell = cell.replace('_NON', '')
class_name, confidence = self.classification_results.get(base_cell, ('unknown', 0.0))
# base_cell = 'A1' → Finds the classification! ✅
```

### Fix 2: Fallback to Locked Classification ✅

If cell lookup fails, use the locked classification from pickup:

```python
# Fallback to locked classification if available
if (class_name == 'unknown' or confidence == 0.0) and self._locked_classification:
    class_name, confidence = self._locked_classification
    self.log(f"[WEBHOOK] Using locked classification: {class_name} ({confidence:.0%})")
```

### Fix 3: Better Logging ✅

Added detailed logging to help debug:

```python
if class_name == 'unknown' or confidence == 0.0:
    self.log(f"⚠ No classification data for cell {base_cell}, skipping webhook")
    self.log(f"   Available cells: {list(self.classification_results.keys())}")
    return

# On error, show full traceback
except Exception as e:
    self.log(f"⚠ Post-pickup webhook error: {e}")
    import traceback
    self.log(f"   Details: {traceback.format_exc()}")
```

## How It Works Now

```
Pickup completes for A1_NON
    ↓
send_webhook_after_pickup('A1_NON')
    ↓
Strip suffix: base_cell = 'A1'
    ↓
Lookup: classification_results.get('A1')
    ↓
Found: ('non-biodegradable', 0.87) ✅
    ↓
Send webhook to Laravel ✅
    ↓
Log: "✓ Webhook sent: non-biodegradable (87%)"
```

### Fallback Flow

```
Pickup completes for A1_NON
    ↓
Strip suffix: base_cell = 'A1'
    ↓
Lookup: classification_results.get('A1')
    ↓
NOT FOUND → ('unknown', 0.0) ❌
    ↓
Fallback: Use _locked_classification
    ↓
Found: ('non-biodegradable', 0.87) ✅
    ↓
Log: "[WEBHOOK] Using locked classification: non-biodegradable (87%)"
    ↓
Send webhook to Laravel ✅
```

## What You'll See in Logs

### Success Case:
```
Pickup for A1_NON complete! (Total: 12.456s)
Object picked up successfully!
✓ Webhook sent: non-biodegradable (87%)
📊 Capacity sent: BIO=45%, NON-BIO=30%
```

### Fallback Case:
```
Pickup for A1_NON complete! (Total: 12.456s)
Object picked up successfully!
[WEBHOOK] Using locked classification: non-biodegradable (87%)
✓ Webhook sent: non-biodegradable (87%)
📊 Capacity sent: BIO=45%, NON-BIO=30%
```

### Failure Case (with details):
```
Pickup for A1_NON complete! (Total: 12.456s)
Object picked up successfully!
⚠ No classification data for cell A1, skipping webhook
   Available cells: ['B2', 'C3']
```

## Testing

### Test 1: Normal Flow
1. Place object in grid cell (e.g., A1)
2. Wait for classification (red/green box)
3. Wait for auto-pickup
4. **Check logs for:** `✓ Webhook sent: ...`
5. **Verify:** Data appears in Laravel database

### Test 2: NON-BIO Object
1. Place non-biodegradable object
2. Wait for red box and "NON-BIO confirmed"
3. Auto-pickup should trigger
4. **Check logs for:** Cell name stripping works (`A1_NON` → `A1`)
5. **Verify:** Webhook sends correctly

### Test 3: Check Fallback
1. If primary lookup fails, check for:
   ```
   [WEBHOOK] Using locked classification: ...
   ```
2. **Verify:** Webhook still sends even if cell lookup fails

## Benefits

✅ **Reliable Sending**: Cell name mismatch no longer prevents webhook  
✅ **Fallback Mechanism**: Locked classification as backup  
✅ **Better Debugging**: Detailed logs show exactly what happened  
✅ **Error Visibility**: Full stack traces on failures  
✅ **Cell Tracking**: Shows available cells when lookup fails  

## Summary

| Issue | Before | After |
|-------|--------|-------|
| **Cell Lookup** | Fails on A1_NON | Strips suffix ✅ |
| **Fallback** | None | Uses locked classification ✅ |
| **Error Logging** | Minimal | Full details + traceback ✅ |
| **Debug Info** | None | Shows available cells ✅ |
| **Reliability** | Sometimes fails | Always tries to send ✅ |

The webhook should now send reliably after every pickup! 🎉
