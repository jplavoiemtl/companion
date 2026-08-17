"""
ESP32 Image Proxy Server
Minimal Flask web server to proxy image requests to Node-RED
Runs behind Synology reverse proxy for HTTPS termination
Provides rate limiting and token authentication

UPDATED:
- Added /esp32home/latest, /esp32home/new, /esp32home/back endpoints
- Added /esp32/live for the live video feed: its own rate limit and its own
  rate-limit bucket, and it forwards height/quality to Node-RED
- Preserved original structure and comments
"""

from flask import Flask, Response, request, jsonify
import requests
from functools import wraps
from collections import defaultdict
import time
import os
import logging

app = Flask(__name__)

# Configuration - use environment variables for Docker
NODE_RED_HOST = os.getenv('NODE_RED_HOST', '192.168.1.130')
NODE_RED_PORT = os.getenv('NODE_RED_PORT', '1880')
API_TOKEN = os.getenv('API_TOKEN', 'CHANGE-THIS-TO-LONG-RANDOM-TOKEN')
RATE_LIMIT = int(os.getenv('RATE_LIMIT', '10'))  # requests per minute per IP
NODE_RED_TIMEOUT = int(os.getenv('NODE_RED_TIMEOUT', '15'))  # seconds

# The live video feed polls a few frames per second, so it needs its own much
# higher limit. 3 fps is 180 requests/minute sustained. 400 leaves room for the
# panel running faster than expected, or two viewing sessions overlapping in the
# same sliding window, while still bounding the endpoint rather than allowing an
# unlimited stream.
RATE_LIMIT_LIVE = int(os.getenv('RATE_LIMIT_LIVE', '400'))  # requests per minute per IP

NODE_RED_BASE = f"http://{NODE_RED_HOST}:{NODE_RED_PORT}"

# Rate limiting storage. Keyed by (ip, bucket) so a live session cannot exhaust
# the allowance for the still-image endpoints - without separate buckets, one
# minute of video would lock out the Latest/Back buttons.
request_counts = defaultdict(list)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def get_client_ip():
    """Get real client IP (handles reverse proxy headers)"""
    # Synology reverse proxy adds X-Forwarded-For header
    if request.headers.get('X-Forwarded-For'):
        # First IP in the chain is the real client
        return request.headers.get('X-Forwarded-For').split(',')[0].strip()
    return request.remote_addr


def check_rate_limit(ip_address, limit, bucket='default'):
    """Check if IP has exceeded the rate limit for this bucket"""
    now = time.time()
    minute_ago = now - 60
    key = (ip_address, bucket)

    # Clean old entries
    request_counts[key] = [
        timestamp for timestamp in request_counts[key]
        if timestamp > minute_ago
    ]

    # Debug logging. Skipped for the live bucket: at a few frames per second it
    # would produce hundreds of lines a minute and bury everything else.
    if bucket != 'live':
        logger.info(f"Rate limit check for {ip_address} [{bucket}]: {len(request_counts[key])}/{limit} requests in last minute")

    # Check limit
    if len(request_counts[key]) >= limit:
        return False

    # Add current request
    request_counts[key].append(now)
    return True


def require_token(func=None, *, limit=None, bucket='default'):
    """Decorator to validate API token.

    Usable bare or with arguments:
        @require_token
        @require_token(limit=RATE_LIMIT_LIVE, bucket='live')
    """
    def decorator(f):
        @wraps(f)
        def decorated_function(*args, **kwargs):
            client_ip = get_client_ip()
            effective_limit = RATE_LIMIT if limit is None else limit

            if bucket != 'live':
                logger.info(f"Client IP detected as: {client_ip}")

            # Check rate limit first
            if not check_rate_limit(client_ip, effective_limit, bucket):
                logger.warning(f"Rate limit exceeded for IP: {client_ip} [{bucket}]")
                return jsonify({"error": f"Rate limit exceeded. Max {effective_limit} requests per minute."}), 429

            # Check token
            token = request.args.get('token')
            if not token or token != API_TOKEN:
                logger.warning(f"Invalid token attempt from IP: {client_ip}, Token: {token[:8] if token else 'None'}...")
                return jsonify({"error": "Invalid or missing token"}), 401

            if bucket != 'live':
                logger.info(f"Valid request from IP: {client_ip} to {request.path}")
            return f(*args, **kwargs)

        return decorated_function

    # Bare @require_token
    if func is not None:
        return decorator(func)
    # @require_token(...)
    return decorator


# ---------------------------------------------------------------------------
# ESP32 CAMERA ENDPOINTS (ORIGINAL)
# ---------------------------------------------------------------------------

@app.route('/esp32/latest')
@require_token
def get_latest():
    """Get latest image from Node-RED"""
    try:
        logger.info(f"Requesting latest image from Node-RED: {NODE_RED_BASE}/esp32/latest")
        response = requests.get(f"{NODE_RED_BASE}/esp32/latest", timeout=NODE_RED_TIMEOUT)

        if response.status_code == 200:
            logger.info(f"Successfully retrieved image, size: {len(response.content)} bytes")
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status: {response.status_code}")
            return jsonify({"error": "Failed to get image from Node-RED"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED request timed out")
        return jsonify({"error": "Node-RED timeout"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


@app.route('/esp32/new')
@require_token
def capture_new():
    """Trigger new image capture (may take 2-5 seconds)"""
    try:
        logger.info(f"Requesting new image capture from Node-RED: {NODE_RED_BASE}/esp32/new")
        response = requests.get(f"{NODE_RED_BASE}/esp32/new", timeout=NODE_RED_TIMEOUT)

        if response.status_code == 200:
            logger.info(f"Successfully captured new image, size: {len(response.content)} bytes")
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status: {response.status_code}")
            return jsonify({"error": "Failed to capture new image"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED capture request timed out")
        return jsonify({"error": "Capture timeout - camera may be busy"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


@app.route('/esp32/back')
@require_token
def get_previous():
    """Get previous image from Node-RED"""
    try:
        logger.info(f"Requesting previous image from Node-RED: {NODE_RED_BASE}/esp32/back")
        response = requests.get(f"{NODE_RED_BASE}/esp32/back", timeout=NODE_RED_TIMEOUT)

        if response.status_code == 200:
            logger.info(f"Successfully retrieved previous image, size: {len(response.content)} bytes")
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status: {response.status_code}")
            return jsonify({"error": "Failed to get previous image"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED request timed out")
        return jsonify({"error": "Node-RED timeout"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


# ---------------------------------------------------------------------------
# LIVE VIDEO FEED (NEW)
# One frame per request, polled at a few frames per second by the companion.
# Differs from the endpoints above in three ways:
#   - its own, much higher rate limit
#   - its own rate-limit bucket, so it cannot starve the still endpoints
#   - it forwards height and quality through to Node-RED, which clamps them
# Per-request logging is deliberately omitted; only failures are logged.
# ---------------------------------------------------------------------------

@app.route('/esp32/live')
@require_token(limit=RATE_LIMIT_LIVE, bucket='live')
def get_live_frame():
    """Get a single live frame (Frigate snapshot, proxied via Node-RED)"""
    # Only forward the parameters we know about. Node-RED clamps the values.
    params = {}
    if request.args.get('height'):
        params['height'] = request.args.get('height')
    if request.args.get('quality'):
        params['quality'] = request.args.get('quality')

    try:
        response = requests.get(
            f"{NODE_RED_BASE}/esp32/live",
            params=params,
            timeout=NODE_RED_TIMEOUT
        )

        if response.status_code == 200:
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status for live frame: {response.status_code}")
            return jsonify({"error": "Failed to get live frame"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED live frame request timed out")
        return jsonify({"error": "Node-RED timeout"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED for live frame: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


# ---------------------------------------------------------------------------
# ESP32 HOME CAMERA ENDPOINTS (NEW)
# Same behavior as ESP32 endpoints, different Node-RED paths
# ---------------------------------------------------------------------------

@app.route('/esp32home/latest')
@require_token
def get_home_latest():
    """Get latest home image from Node-RED"""
    try:
        logger.info(f"Requesting latest HOME image from Node-RED: {NODE_RED_BASE}/esp32home/latest")
        response = requests.get(f"{NODE_RED_BASE}/esp32home/latest", timeout=NODE_RED_TIMEOUT)

        if response.status_code == 200:
            logger.info(f"Successfully retrieved HOME image, size: {len(response.content)} bytes")
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status: {response.status_code}")
            return jsonify({"error": "Failed to get home image"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED HOME request timed out")
        return jsonify({"error": "Node-RED timeout"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


@app.route('/esp32home/new')
@require_token
def capture_home_new():
    """Trigger new home image capture"""
    try:
        logger.info(f"Requesting new HOME image capture from Node-RED: {NODE_RED_BASE}/esp32home/new")
        response = requests.get(f"{NODE_RED_BASE}/esp32home/new", timeout=NODE_RED_TIMEOUT)

        if response.status_code == 200:
            logger.info(f"Successfully captured new HOME image, size: {len(response.content)} bytes")
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status: {response.status_code}")
            return jsonify({"error": "Failed to capture new home image"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED HOME capture request timed out")
        return jsonify({"error": "Capture timeout - camera may be busy"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


@app.route('/esp32home/back')
@require_token
def get_home_previous():
    """Get previous home image from Node-RED"""
    try:
        logger.info(f"Requesting previous HOME image from Node-RED: {NODE_RED_BASE}/esp32home/back")
        response = requests.get(f"{NODE_RED_BASE}/esp32home/back", timeout=NODE_RED_TIMEOUT)

        if response.status_code == 200:
            logger.info(f"Successfully retrieved previous HOME image, size: {len(response.content)} bytes")
            return Response(
                response.content,
                mimetype='image/jpeg',
                headers={
                    'Content-Type': 'image/jpeg',
                    'Cache-Control': 'no-cache, no-store, must-revalidate'
                }
            )
        else:
            logger.error(f"Node-RED returned status: {response.status_code}")
            return jsonify({"error": "Failed to get previous home image"}), response.status_code

    except requests.exceptions.Timeout:
        logger.error("Node-RED HOME request timed out")
        return jsonify({"error": "Node-RED timeout"}), 504
    except requests.exceptions.RequestException as e:
        logger.error(f"Error connecting to Node-RED: {str(e)}")
        return jsonify({"error": "Node-RED unavailable"}), 503


# ---------------------------------------------------------------------------
# SERVICE / HEALTH ENDPOINTS
# ---------------------------------------------------------------------------

@app.route('/health')
def health_check():
    """Health check endpoint (no token required)"""
    return jsonify({
        "status": "healthy",
        "rate_limit": f"{RATE_LIMIT} requests/minute",
        "rate_limit_live": f"{RATE_LIMIT_LIVE} requests/minute"
    }), 200


@app.route('/')
def root():
    """Root endpoint - basic info"""
    return jsonify({
        "service": "ESP32 Image Proxy",
        "endpoints": [
            "/esp32/latest",
            "/esp32/new",
            "/esp32/back",
            "/esp32/live",
            "/esp32home/latest",
            "/esp32home/new",
            "/esp32home/back"
        ],
        "auth": "Token required in query parameter: ?token=YOUR_TOKEN"
    }), 200


if __name__ == '__main__':
    logger.info("=" * 60)
    logger.info("ESP32 Image Proxy Server Starting")
    logger.info("=" * 60)
    logger.info(f"Node-RED target: {NODE_RED_BASE}")
    logger.info(f"Rate limit: {RATE_LIMIT} requests/minute per IP")
    logger.info(f"Rate limit (live): {RATE_LIMIT_LIVE} requests/minute per IP")
    logger.info(f"Request timeout: {NODE_RED_TIMEOUT} seconds")
    logger.info(f"Token configured: {'Yes' if API_TOKEN != 'CHANGE-THIS-TO-LONG-RANDOM-TOKEN' else 'NO - CHANGE IT!'}")
    logger.info("=" * 60)

    # Run Flask server (HTTP only - HTTPS handled by Synology)
    app.run(host='0.0.0.0', port=5000, debug=False)
