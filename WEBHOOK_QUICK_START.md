# Waste Classification Webhook - Quick Start Guide

## What Was Added

A new feature that automatically sends waste classification data to your Laravel website when the robotic arm classifies waste objects.

## New Files Created

1. **`code/classification_webhook.py`** - Webhook client that sends data to Laravel
2. **`code/test_webhook.py`** - Test script to verify webhook functionality
3. **`WEBHOOK_README.md`** - Complete documentation
4. **`WEBHOOK_QUICK_START.md`** - This file

## Modified Files

1. **`code/unified_control.py`** - Integrated webhook into classification system

## How to Use

### Step 1: Test the Webhook

```bash
cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision/code
python3 test_webhook.py
```

This will test both local and production endpoints.

### Step 2: Run the Main Application

```bash
cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision
python3 code/unified_control.py
```

### Step 3: Enable Waste Classification

1. Go to **Grid Calibration** tab
2. Click **"Enable Waste Classification"** checkbox
3. You'll see: "✓ Webhook client initialized"
4. The button next to it shows: "Webhook: Local"

### Step 4: Switch Endpoint (Optional)

Click the **"Webhook: Local"** button to switch to Production mode.
- **Local**: `http://localhost:8000` (for testing)
- **Production**: `https://smartrecyclebot-python.onrender.com` (for live)

### Step 5: Classify Objects

When objects are detected and classified:
- System collects 10 samples
- Confirms classification (6/10 majority)
- Automatically sends to Laravel website
- Log shows: "✓ Webhook sent: biodegradable (95%)"

## Requirements Checklist

For webhook to work:

- ✅ Internet connection (for production)
- ✅ API key (already configured)
- ✅ Website endpoint accessible
- ✅ Laravel service running

## What Gets Sent

```json
{
  "bin_id": 1,
  "classification": "biodegradable",
  "score": 0.9523,
  "model_name": "waste_inference_v1"
}
```

Stored in `waste_objects` table in your Laravel database.

## Troubleshooting

### "Connection failed"
- Local: Run `php artisan serve` on port 8000
- Production: Wake up your Render.com services

### "Failed to initialize"
- Check `requests` library: `pip install requests`
- Verify file exists: `code/classification_webhook.py`

### No data in database
- Check Laravel logs
- Verify API endpoint exists
- Confirm API key matches

## Quick Test Commands

```bash
# Test webhook module
python3 code/classification_webhook.py

# Run full test suite
python3 code/test_webhook.py

# Check if requests is installed
pip list | grep requests
```

## Next Steps

1. Test with local Laravel server first
2. Verify data appears in database
3. Switch to production endpoint
4. Monitor logs for successful sends

## Support

See `WEBHOOK_README.md` for complete documentation.
