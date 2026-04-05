#!/usr/bin/env python3
"""
Webhook Client for sending classification data to Laravel website
Handles communication with both local and production endpoints
"""

import requests
import threading
import time
import logging
from typing import Optional, Dict, Any

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ClassificationWebhookClient:
    """Sends waste classification data to Laravel website webhook"""

    # Webhook configuration
    LOCAL_API_URL = "http://localhost:8000/api/waste-objects/webhook"
    PRODUCTION_API_URL = "https://smartrecyclebot-b86k.onrender.com/api/waste-objects/webhook"
    API_KEY = "9kX7mP2nQ8vL4sR6wT1yF3hJ5gB0dZ9c"

    # Bin ID mapping
    BIN_ID_MAP = {
        'biodegradable': 1,
        'non-biodegradable': 2
    }

    def __init__(self, use_production: bool = False, timeout: float = 5.0):
        """Initialize webhook client

        Args:
            use_production: If True, use production URL. Otherwise use local.
            timeout: Request timeout in seconds
        """
        self.use_production = use_production
        self.timeout = timeout
        self.base_url = self.PRODUCTION_API_URL if use_production else self.LOCAL_API_URL
        self._lock = threading.Lock()
        self._last_send_time = 0
        self._send_cooldown = 1.0  # Minimum 1 second between sends to avoid spam

        logger.info(f"Webhook client initialized - {'Production' if use_production else 'Local'} mode")
        logger.info(f"Base URL: {self.base_url}")

    def send_classification(self, 
                            classification: str, 
                            confidence: float,
                            model_name: Optional[str] = None,
                            bin_id: Optional[int] = None) -> Dict[str, Any]:
        """Send classification data to webhook

        Args:
            classification: 'biodegradable' or 'non-biodegradable'
            confidence: Classification confidence (0.0 to 1.0)
            model_name: Optional model name/identifier
            bin_id: Optional bin ID override (1 or 2)

        Returns:
            dict: Response data with 'success' boolean and 'message' string
        """
        # Check cooldown
        current_time = time.time()
        if current_time - self._last_send_time < self._send_cooldown:
            return {
                'success': False,
                'message': 'Rate limited - please wait before sending again'
            }

        # Validate classification
        if classification not in self.BIN_ID_MAP:
            return {
                'success': False,
                'message': f'Invalid classification: {classification}'
            }

        # Determine bin_id
        if bin_id is None:
            bin_id = self.BIN_ID_MAP.get(classification)

        # Prepare payload
        payload = {
            'bin_id': bin_id,
            'classification': classification,
            'score': round(float(confidence), 4),
        }

        # Add model_name if provided (nullable)
        if model_name:
            payload['model_name'] = model_name

        # Send in a separate thread to avoid blocking
        thread = threading.Thread(
            target=self._send_request,
            args=(payload,),
            daemon=True
        )
        thread.start()

        return {
            'success': True,
            'message': f'Classification data queued for sending: {classification} ({confidence:.4f})'
        }

    def _send_request(self, payload: Dict[str, Any]):
        """Actually send the HTTP request (runs in separate thread)

        Args:
            payload: Data to send
        """
        with self._lock:
            self._last_send_time = time.time()

        headers = {
            'Content-Type': 'application/json',
            'X-API-Key': self.API_KEY
        }

        url = f"{self.base_url}"

        try:
            logger.info(f"Sending classification to {url}: {payload}")

            response = requests.post(
                url,
                json=payload,
                headers=headers,
                timeout=self.timeout
            )

            if response.status_code in [200, 201]:
                logger.info(f"✓ Classification sent successfully")
                try:
                    data = response.json()
                    logger.debug(f"Response: {data}")
                except:
                    pass
            else:
                logger.warning(f"✗ Server returned status {response.status_code}: {response.text[:200]}")

        except requests.exceptions.ConnectionError as e:
            logger.warning(f"✗ Connection failed - website may be offline: {str(e)}")
        except requests.exceptions.Timeout as e:
            logger.warning(f"✗ Request timed out: {str(e)}")
        except requests.exceptions.RequestException as e:
            logger.warning(f"✗ Request failed: {str(e)}")
        except Exception as e:
            logger.error(f"✗ Unexpected error sending webhook: {str(e)}")

    def test_connection(self) -> Dict[str, Any]:
        """Test connection to the webhook endpoint

        Returns:
            dict: Test result with 'success' and 'message'
        """
        test_payload = {
            'bin_id': 1,
            'classification': 'biodegradable',
            'score': 0.9999,
            'model_name': 'test'
        }

        headers = {
            'Content-Type': 'application/json',
            'X-API-Key': self.API_KEY
        }

        try:
            response = requests.post(
                self.base_url,
                json=test_payload,
                headers=headers,
                timeout=self.timeout
            )

            if response.status_code == 200:
                return {
                    'success': True,
                    'message': f'✓ Connection successful to {self.base_url}'
                }
            else:
                return {
                    'success': False,
                    'message': f'✗ Server returned {response.status_code}: {response.text[:200]}'
                }

        except requests.exceptions.ConnectionError:
            return {
                'success': False,
                'message': f'✗ Cannot connect to {self.base_url} - website may be offline'
            }
        except Exception as e:
            return {
                'success': False,
                'message': f'✗ Connection test failed: {str(e)}'
            }

    def update_endpoint(self, use_production: bool):
        """Switch between local and production endpoints

        Args:
            use_production: True for production, False for local
        """
        self.use_production = use_production
        self.base_url = self.PRODUCTION_API_URL if use_production else self.LOCAL_API_URL
        logger.info(f"Endpoint switched to {'Production' if use_production else 'Local'}: {self.base_url}")


# Convenience function for simple usage
def send_classification_simple(classification: str, confidence: float, 
                               model_name: str = None, use_production: bool = False):
    """Simple function to send classification data

    Args:
        classification: 'biodegradable' or 'non-biodegradable'
        confidence: Confidence score (0.0 to 1.0)
        model_name: Optional model name
        use_production: True for production, False for local
    """
    client = ClassificationWebhookClient(use_production=use_production)
    return client.send_classification(
        classification=classification,
        confidence=confidence,
        model_name=model_name
    )


if __name__ == "__main__":
    """Test the webhook client"""
    print("Testing Classification Webhook Client...")
    print("=" * 60)

    # Test with local endpoint
    client = ClassificationWebhookClient(use_production=False)

    print("\n1. Testing connection...")
    result = client.test_connection()
    print(f"   {result['message']}")

    print("\n2. Sending test classification...")
    result = client.send_classification(
        classification='biodegradable',
        confidence=0.9523,
        model_name='waste_inference_v1'
    )
    print(f"   {result['message']}")

    print("\n3. Sending another test...")
    result = client.send_classification(
        classification='non-biodegradable',
        confidence=0.8741,
        model_name='waste_inference_v1'
    )
    print(f"   {result['message']}")

    print("\n" + "=" * 60)
    print("Test complete! Check your Laravel website for received data.")
    print("Note: If connection failed, ensure the website is running.")
