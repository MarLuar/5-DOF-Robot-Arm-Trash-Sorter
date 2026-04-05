#!/usr/bin/env python3
"""
Test script for waste classification webhook
Tests connection and sends sample data to verify the integration works
"""

import sys
import os

# Add parent directory to path to import webhook
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from classification_webhook import ClassificationWebhookClient
import time

def main():
    print("=" * 70)
    print("WASTE CLASSIFICATION WEBHOOK TEST")
    print("=" * 70)
    
    # Test 1: Local endpoint
    print("\n[TEST 1] Testing LOCAL endpoint (http://localhost:8000)")
    print("-" * 70)
    
    local_client = ClassificationWebhookClient(use_production=False)
    
    print("1.1 Testing connection...")
    result = local_client.test_connection()
    print(f"    {result['message']}")
    
    if result['success']:
        print("1.2 Sending sample biodegradable classification...")
        result = local_client.send_classification(
            classification='biodegradable',
            confidence=0.9523,
            model_name='test_model_v1'
        )
        print(f"    {result['message']}")
        
        time.sleep(1.5)  # Wait for cooldown
        
        print("1.3 Sending sample non-biodegradable classification...")
        result = local_client.send_classification(
            classification='non-biodegradable',
            confidence=0.8741,
            model_name='test_model_v1'
        )
        print(f"    {result['message']}")
    else:
        print("    ⚠ Local endpoint not available - this is OK if Laravel isn't running locally")
    
    # Test 2: Production endpoint
    print("\n[TEST 2] Testing PRODUCTION endpoint (https://smartrecyclebot-python.onrender.com)")
    print("-" * 70)
    
    prod_client = ClassificationWebhookClient(use_production=True)
    
    print("2.1 Testing connection...")
    result = prod_client.test_connection()
    print(f"    {result['message']}")
    
    if result['success']:
        print("2.2 Sending sample biodegradable classification...")
        result = prod_client.send_classification(
            classification='biodegradable',
            confidence=0.9234,
            model_name='test_model_v1'
        )
        print(f"    {result['message']}")
        
        time.sleep(1.5)  # Wait for cooldown
        
        print("2.3 Sending sample non-biodegradable classification...")
        result = prod_client.send_classification(
            classification='non-biodegradable',
            confidence=0.7856,
            model_name='test_model_v1'
        )
        print(f"    {result['message']}")
    else:
        print("    ⚠ Production endpoint not available - check internet connection and Render.com status")
    
    # Test 3: Rate limiting
    print("\n[TEST 3] Testing rate limiting...")
    print("-" * 70)
    
    print("3.1 Sending rapid requests (should be rate limited)...")
    client = ClassificationWebhookClient(use_production=False)
    
    result1 = client.send_classification('biodegradable', 0.9)
    print(f"    Request 1: {result1['message']}")
    
    result2 = client.send_classification('non-biodegradable', 0.8)
    print(f"    Request 2: {result2['message']}")
    
    result3 = client.send_classification('biodegradable', 0.95)
    print(f"    Request 3: {result3['message']}")
    
    # Test 4: Invalid classification
    print("\n[TEST 4] Testing invalid classification...")
    print("-" * 70)
    
    result = client.send_classification('invalid_type', 0.5)
    print(f"    Result: {result['message']}")
    
    print("\n" + "=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)
    print("\nNotes:")
    print("- Check your Laravel website database for received classification data")
    print("- Local test requires: php artisan serve running on localhost:8000")
    print("- Production test requires: Internet + Render.com services awake")
    print("- Rate limiting prevents spam (1 second cooldown)")
    print("\nIf connection tests failed, ensure:")
    print("  1. Website/service is running and accessible")
    print("  2. API endpoint exists: /api/classification-receive")
    print("  3. API key is configured correctly in Laravel")
    print("=" * 70)

if __name__ == "__main__":
    main()
