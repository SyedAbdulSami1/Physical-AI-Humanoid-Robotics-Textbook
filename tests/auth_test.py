import pytest
import httpx
import asyncio

# Base URL for the local server
BASE_URL = "http://127.0.0.1:8000"

# Test cases for authentication flow
AUTH_TEST_CASES = [
    {
        "email": "test@example.com",
        "password": "securepassword123",
        "name": "Test User",
        "profile": {
            "softwareExperience": "intermediate",
            "hardwareExperience": "beginner"
        }
    }
]

@pytest.fixture
async def client():
    return httpx.AsyncClient()

@pytest.mark.asyncio
async def test_auth_flow(client):
    """Test the complete authentication flow: signup -> login -> profile access"""
    
    for test_case in AUTH_TEST_CASES:
        try:
            # Test signup
            print(f"Testing signup for {test_case['email']}")
            response = await client.post(f"{BASE_URL}/auth/register", json={
                "email": test_case["email"],
                "password": test_case["password"],
                "name": test_case["name"],
                "profile": test_case["profile"]
            })
            assert response.status_code == 201, f"Signup failed with status {response.status_code}"
            print("Signup successful")

            # Test login
            print(f"Testing login for {test_case['email']}")
            response = await client.post(f"{BASE_URL}/auth/login", data={
                "username": test_case["email"],
                "password": test_case["password"]
            })
            assert response.status_code == 200, f"Login failed with status {response.status_code}"
            token = response.json().get("access_token")
            assert token, "Access token not found in login response"
            print("Login successful")

            # Test profile access/update
            print(f"Testing profile access with token: {token}")
            headers = {"Authorization": f"Bearer {token}"}
            response = await client.get(f"{BASE_URL}/auth/profile", headers=headers)
            assert response.status_code == 200, f"Profile access failed with status {response.status_code}"
            profile_data = response.json()
            assert profile_data.get("email") == test_case["email"], "Profile data mismatch"
            print("Profile access successful")

        except Exception as e:
            pytest.fail(f"Auth flow test failed: {str(e)}")

@pytest.mark.asyncio
async def test_auth_security(client):
    """Test authentication security measures"""
    try:
        # Try to access protected endpoint without token
        response = await client.get(f"{BASE_URL}/auth/profile")
        # Expecting a 401 Unauthorized error
        assert response.status_code == 401, "Unauthorized access should be denied"
        print("Auth security tests passed")
    except Exception as e:
        pytest.fail(f"Auth security test failed: {str(e)}")

if __name__ == "__main__":
    # To run this test, ensure the FastAPI server is running on port 8000
    # Command: uvicorn app.main:app --host 127.0.0.1 --port 8000
    pytest.main([__file__])