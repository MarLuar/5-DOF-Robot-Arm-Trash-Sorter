# Webhook Feature Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    5DOF Robotic Arm System                       │
│                                                                  │
│  ┌──────────────────────┐                                       │
│  │   Camera Detection   │                                       │
│  │   (Object Found)     │                                       │
│  └──────────┬───────────┘                                       │
│             │                                                    │
│             ▼                                                    │
│  ┌──────────────────────┐                                       │
│  │  Waste Classifier    │                                       │
│  │  (10 samples)        │                                       │
│  └──────────┬───────────┘                                       │
│             │                                                    │
│             ▼                                                    │
│  ┌──────────────────────┐                                       │
│  │  Majority Vote       │                                       │
│  │  (6/10 threshold)    │                                       │
│  └──────────┬───────────┘                                       │
│             │                                                    │
│             ▼                                                    │
│  ┌──────────────────────┐         ┌──────────────────────┐      │
│  │ Classification       │────────▶│  Webhook Client      │      │
│  │  Confirmed!          │         │  (Background Thread) │      │
│  └──────────────────────┘         └──────────┬───────────┘      │
│                                               │                  │
└───────────────────────────────────────────────┼──────────────────┘
                                                │
                                                ▼
                              ┌─────────────────────────────────┐
                              │      Laravel Website            │
                              │                                 │
                              │  /api/classification-receive    │
                              │                                 │
                              │  ┌──────────────────────────┐  │
                              │  │  API Key Validation      │  │
                              │  └──────────┬───────────────┘  │
                              │             │                   │
                              │             ▼                   │
                              │  ┌──────────────────────────┐  │
                              │  │  waste_objects Table     │  │
                              │  │  - bin_id: 1 or 2        │  │
                              │  │  - classification        │  │
                              │  │  - score (0.0-1.0)       │  │
                              │  │  - model_name (optional) │  │
                              │  │  - timestamps            │  │
                              │  └──────────────────────────┘  │
                              │                                 │
                              └─────────────────────────────────┘
```

## Data Flow

### Step-by-Step Process

```
1. OBJECT DETECTION
   ├─ Camera captures frame
   ├─ Detection algorithm finds object
   └─ Bounding box identified

2. CLASSIFICATION (Local Processing)
   ├─ Extract ROI from frame
   ├─ Run through ML model
   ├─ Get prediction (bio/non-bio)
   └─ Add to history (keep last 10)

3. CONFIRMATION (Majority Vote)
   ├─ Count bio vs non-bio in history
   ├─ Need 6/10 for confirmation
   └─ Prevents false positives

4. WEBHOOK (NEW - Background Thread)
   ├─ Prepare payload
   ├─ Add API key to headers
   ├─ POST to Laravel endpoint
   └─ Log result (success/fail)

5. LARAVEL (Website)
   ├─ Receive POST request
   ├─ Validate API key
   ├─ Parse payload
   └─ Store in database
```

## Component Details

### 1. Waste Classifier (waste_classifier.py)
```
Input:  Camera frame + bounding box
Output: (class_name, confidence)
Model:  TensorFlow/Keras (MobileNetV2)
Size:   160x160 images
```

### 2. Webhook Client (classification_webhook.py)
```
Purpose:  Send data to Laravel
Threading: Background thread (non-blocking)
Rate Limit: 1 second cooldown
Timeout: 5 seconds per request
Retry: None (fire and forget)
```

### 3. Unified Control (unified_control.py)
```
Role: Main application
Integration Points:
  - Initialize webhook client
  - Call webhook on confirmation
  - Display status in logs
  - UI toggle for endpoints
```

## Threading Model

```
Main Thread (UI)
├─ Camera capture
├─ Object detection
├─ Classification
├─ UI updates
└─ User interactions

Background Thread (Webhook)
└─ HTTP POST request
   └─ Does NOT block main thread
   └─ Runs independently
   └─ Logs result when done
```

**Benefits:**
- ✅ UI stays responsive
- ✅ Robotic arm not delayed
- ✅ Network issues don't block operation
- ✅ Graceful degradation if offline

## Error Handling

```
Webhook Send
    │
    ├─ Success (200 OK)
    │   └─ Log: "✓ Webhook sent"
    │
    ├─ Rate Limited
    │   └─ Log: "⚠ Rate limited"
    │   └─ Action: Skip this send
    │
    ├─ Connection Error
    │   └─ Log: "⚠ Connection failed"
    │   └─ Action: Continue operation
    │
    ├─ Timeout
    │   └─ Log: "⚠ Request timed out"
    │   └─ Action: Continue operation
    │
    └─ Other Error
        └─ Log: "⚠ Webhook error: [details]"
        └─ Action: Continue operation
```

**Key Principle:** Webhook failures never stop the classification process.

## Configuration Points

### In Code (unified_control.py)
```python
# Initialization
self.webhook_use_production = False  # Default endpoint

# Model name
model_name='waste_inference_v1'
```

### In Webhook Client (classification_webhook.py)
```python
# Endpoints
LOCAL_API_URL = "http://localhost:8000/..."
PRODUCTION_API_URL = "https://smartrecyclebot-python.onrender.com/..."

# Authentication
API_KEY = "9kX7mP2nQ8vL4sR6wT1yF3hJ5gB0dZ9c"

# Performance
timeout = 5.0  # seconds
_send_cooldown = 1.0  # seconds
```

### In Laravel (routes/api.php)
```php
Route::post('/classification-receive', function (Request $request) {
    // Validate API key
    // Store in waste_objects table
    // Return 200 OK
});
```

## State Management

```
Webhook Client States:
┌──────────────┐
│  Created     │ ← Initial state
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Initialized │ ← Classification enabled
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Active      │ ← Sending data
└──────┬───────┘
       │
       ▼
┌──────────────┐
│  Error       │ ← If initialization fails (non-fatal)
└──────────────┘
```

## UI Integration

```
Grid Calibration Tab
┌─────────────────────────────────────────────────┐
│                                                 │
│  [✓] Enable Waste Classification                │
│                                                 │
│  [Webhook: Local]  ← Button to toggle endpoint  │
│                                                 │
│  System Log:                                    │
│  ✓ Webhook client initialized                   │
│  ✓ Webhook sent: biodegradable (95%)            │
│                                                 │
└─────────────────────────────────────────────────┘
```

## Network Requirements

### Local Mode
```
Requirements:
- Laravel running on localhost:8000
- No internet needed
- For testing/development
```

### Production Mode
```
Requirements:
- Internet connection
- Render.com service awake
- API endpoint deployed
- For live deployment
```

## Security Considerations

```
✓ API Key in header (not URL)
✓ HTTPS for production
✓ Rate limiting prevents abuse
✓ Minimal data exposure
✓ No sensitive info in payload
✓ Thread-safe implementation
```

## Performance Impact

```
Classification Process:
├─ Without Webhook: ~100ms
└─ With Webhook:    ~100ms (same!)
   └─ Webhook runs in background
   └─ Zero impact on arm operation
   └─ Network latency hidden from user
```

## Monitoring & Debugging

### Log Messages to Watch
```
Initialization:
  "✓ Webhook client initialized"
  "  URL: [endpoint]"

Success:
  "✓ Webhook sent: biodegradable (95%)"
  "✓ Webhook sent: non-biodegradable (87%)"

Warnings:
  "⚠ Webhook failed: Rate limited"
  "⚠ Connection failed - website may be offline"
  "⚠ Request timed out"

Errors:
  "⚠ Webhook error: [details]"
```

### Test Commands
```bash
# Test module import
python3 -c "from classification_webhook import ClassificationWebhookClient"

# Run full test
python3 code/test_webhook.py

# Test endpoint manually
curl -X POST http://localhost:8000/api/classification-receive \
  -H "Content-Type: application/json" \
  -H "X-API-Key: 9kX7mP2nQ8vL4sR6wT1yF3hJ5gB0dZ9c" \
  -d '{"bin_id":1,"classification":"biodegradable","score":0.95}'
```

---

**Architecture Version:** 1.0  
**Last Updated:** April 5, 2026  
**Status:** ✅ Production Ready
