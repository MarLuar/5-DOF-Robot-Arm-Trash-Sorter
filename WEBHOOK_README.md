# Waste Classification Webhook Integration

This feature automatically sends waste classification data to your Laravel website when the robotic arm classifies an object.

## Features

✅ **Automatic Data Sending**: Sends classification results to your Laravel website  
✅ **Non-Blocking**: Uses background threads so it won't slow down the robotic arm  
✅ **Rate Limited**: Prevents spam with built-in cooldown (1 second minimum between sends)  
✅ **Error Handling**: Gracefully handles offline websites, timeouts, and connection errors  
✅ **Dual Environment Support**: Works with both local and production URLs  

## How It Works

1. When waste classification is enabled in the Grid Calibration tab
2. The system detects an object in the grid
3. After collecting 10 samples and confirming the classification (6/10 majority vote)
4. The classification data is automatically sent to your Laravel website via webhook
5. The webhook runs in a background thread, so it won't block the robotic arm

## Webhook Details

### Endpoint URLs

- **Local**: `http://localhost:8000/api/classification-receive`
- **Production**: `https://smartrecyclebot-python.onrender.com/api/classification-receive`

### Authentication

All requests include the API key in the header:
```
X-API-Key: 9kX7mP2nQ8vL4sR6wT1yF3hJ5gB0dZ9c
```

### Payload Format

```json
{
  "bin_id": 1,
  "classification": "biodegradable",
  "score": 0.9523,
  "model_name": "waste_inference_v1"
}
```

**Field Descriptions:**
- `bin_id`: 1 for biodegradable, 2 for non-biodegradable
- `classification`: "biodegradable" or "non-biodegradable"
- `score`: Confidence score (0.0 to 1.0, rounded to 4 decimals)
- `model_name`: Optional model identifier (can be empty)

## Requirements

For the webhook to successfully send data, the following must be true:

1. ✅ **Internet Connection**: Required for production URL
2. ✅ **API Key**: Already configured (stored in `classification_webhook.py`)
3. ✅ **URL Accessible**: The website endpoint must be reachable
4. ✅ **Website/Service Awake**: Either local Laravel or production service must be running

## Usage

### In the Application

1. Open the Unified Control System
2. Go to **Grid Calibration** tab
3. Check **"Enable Waste Classification"**
4. The webhook client will automatically initialize
5. When objects are detected and classified, data is sent automatically

### Testing the Webhook

You can test the webhook independently:

```bash
cd /home/koogs/Documents/5DOF_Robotic_Arm_Vision/code
python3 classification_webhook.py
```

This will:
- Test connection to the webhook endpoint
- Send sample classification data
- Show success/failure messages

### Switching Between Local and Production

By default, the system uses the **local** URL. To switch to production:

Edit `code/unified_control.py` and find this line in `__init__`:
```python
self.webhook_use_production = False  # Default to local
```

Change to:
```python
self.webhook_use_production = True  # Use production
```

Or, you can add a UI toggle in the calibration tab to switch dynamically (future enhancement).

## Logging

All webhook activity is logged in the System Log:

- **✓ Webhook client initialized** - Webhook is ready
- **✓ Webhook sent: biodegradable (95%)** - Successfully sent
- **⚠ Webhook failed: Rate limited** - Sending too fast (cooldown active)
- **⚠ Connection failed - website may be offline** - Cannot reach endpoint
- **✗ Request timed out** - Server didn't respond in time

## Troubleshooting

### "Connection failed - website may be offline"

**Solution**: 
- For local: Make sure Laravel is running (`php artisan serve`)
- For production: Check that your Render.com services are awake
- Test with: `curl http://localhost:8000/api/classification-receive`

### "Failed to initialize webhook client"

**Solution**:
- Check that `classification_webhook.py` exists in the `code/` directory
- Ensure `requests` library is installed: `pip install requests`

### Webhook sends but no data in database

**Solution**:
- Verify the API endpoint exists in your Laravel routes
- Check Laravel logs for errors
- Verify the API key matches your Laravel configuration
- Check that the `waste_objects` table exists with correct columns

## File Structure

```
5DOF_Robotic_Arm_Vision/
├── code/
│   ├── unified_control.py           # Main control system (modified)
│   └── classification_webhook.py    # Webhook client (NEW)
├── waste_classifier.py              # Classification model
└── WEBHOOK_README.md                # This file
```

## Database Schema

Data is stored in the `waste_objects` table:

| Column | Type | Description |
|--------|------|-------------|
| id | int | Auto-increment primary key |
| bin_id | int | 1 = biodegradable, 2 = non-biodegradable |
| model_name | string (nullable) | Model identifier |
| classification | string | "biodegradable" or "non-biodegradable" |
| score | float | Confidence (4 decimal places) |
| created_at | datetime | Record creation timestamp |
| updated_at | datetime | Record update timestamp |

## Advanced Configuration

### Adjusting Rate Limit

Edit `classification_webhook.py`:
```python
self._send_cooldown = 1.0  # Minimum seconds between sends
```

### Changing Timeout Duration

```python
self.timeout = 5.0  # Request timeout in seconds
```

### Custom Model Name

When calling `send_classification_webhook`, you can customize the model name in `unified_control.py`:

```python
result = self.webhook_client.send_classification(
    classification=classification,
    confidence=confidence,
    model_name='your_custom_model_name'  # Change this
)
```

## Future Enhancements

- [ ] UI toggle to switch between local/production
- [ ] Webhook status indicator (online/offline)
- [ ] Retry failed sends when website comes back online
- [ ] Queue system for offline caching
- [ ] Manual "Test Connection" button in UI
- [ ] Configurable API key and URL via settings file

## Support

If you encounter issues:
1. Check the System Log in the application
2. Run the test script: `python3 classification_webhook.py`
3. Verify Laravel endpoint is accessible
4. Check that all requirements are met (listed above)
