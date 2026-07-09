import importlib.util
import sys
import pytest

spec = importlib.util.spec_from_file_location(
    "app", "watod_scripts/tools/log-viewer/app.py"
)
mod = importlib.util.load_from_spec(spec)
spec.loader.exec_module(mod)
app = mod.app


@pytest.fixture
def client():
    app.config["TESTING"] = True
    with app.test_client() as c:
        yield c


@pytest.mark.parametrize("endpoint,token", [
    ("/api/containers", None),                          # exact exploit: no auth
    ("/api/logs/abc123", "Bearer expired.token.here"),  # expired/malformed token
    ("/api/topic-health", "Bearer "),                   # boundary: empty bearer
    ("/api/containers", "not-a-token"),                 # malformed token
])
def test_protected_endpoints_reject_unauthenticated(client, endpoint, token):
    """Invariant: all API endpoints must reject requests lacking valid credentials with 401 or 403."""
    headers = {"Authorization": token} if token else {}
    response = client.get(endpoint, headers=headers)
    assert response.status_code in (401, 403), (
        f"Endpoint {endpoint!r} with token={token!r} returned {response.status_code}; "
        "expected 401 or 403 — unauthenticated access must be denied."
    )