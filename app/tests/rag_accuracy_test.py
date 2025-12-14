import asyncio
import json
import os
from typing import List, Dict
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

from app.skills.rag_agent import rag_agent_instance

class RAGAccuracyTester:
    """
    Tests the accuracy of the RAG system to ensure >90% accuracy by running it against a predefined set of test cases.
    """
    
    def __init__(self, rag_agent):
        self.rag_agent = rag_agent
        self.test_cases = self._load_test_cases()
    
    def _load_test_cases(self) -> List[Dict]:
        """
        Loads a predefined set of test cases. In a real-world scenario, this would load from a dedicated test dataset file (e.g., a CSV or JSON file).
        
        Each test case includes:
        - query: The question to ask the RAG agent.
        - expected_keywords: A list of essential keywords that a correct answer should contain.
        """
        return [
            {
                "query": "What is forward kinematics in robotics?",
                "expected_keywords": ["kinematics", "position", "motion", "joint angles", "end-effector"],
            },
            {
                "query": "Explain PID controller in humanoid robots",
                "expected_keywords": ["PID", "proportional", "integral", "derivative", "control", "error"],
            },
            {
                "query": "What is sensor fusion in robotics?",
                "expected_keywords": ["sensor fusion", "multiple sensors", "data integration", "accuracy", "reliability"],
            },
            {
                "query": "What is the purpose of a URDF file?",
                "expected_keywords": ["URDF", "robot model", "kinematic", "dynamic", "visual representation"],
            },
            {
                "query": "How does Gazebo simulate physics?",
                "expected_keywords": ["Gazebo", "physics engine", "ODE", "Bullet", "Simbody", "DART", "collision"],
            }
        ]
    
    async def run_accuracy_test(self) -> Dict:
        """
        Executes the accuracy test by iterating through test cases, calling the RAG agent, and evaluating the responses.
        """
        correct_answers = 0
        total_tests = len(self.test_cases)
        
        results = {
            "total_tests": total_tests,
            "correct_answers": 0,
            "accuracy_percentage": 0,
            "detailed_results": []
        }
        
        for i, test_case in enumerate(self.test_cases):
            print(f"Running test {i+1}/{total_tests}: \"{test_case['query']}\"")
            
            try:
                # Get response from the RAG agent by performing a real search
                answer, sources = await self.rag_agent.generate_response(query=test_case["query"])
                
                # Check if the response contains the expected keywords
                is_correct = self._check_response_accuracy(answer, test_case["expected_keywords"])
                
                result_detail = {
                    "query": test_case["query"],
                    "response": answer,
                    "sources": sources,
                    "expected_keywords": test_case["expected_keywords"],
                    "is_correct": is_correct
                }
                
                results["detailed_results"].append(result_detail)
                
                if is_correct:
                    correct_answers += 1
                
                print(f"  Result: {'✓ PASS' if is_correct else '✗ FAIL'}")
                
            except Exception as e:
                print(f"  Error in test {i+1}: {str(e)}")
                results["detailed_results"].append({
                    "query": test_case["query"],
                    "response": None,
                    "sources": [],
                    "expected_keywords": test_case["expected_keywords"],
                    "is_correct": False,
                    "error": str(e)
                })
        
        results["correct_answers"] = correct_answers
        accuracy_percentage = (correct_answers / total_tests) * 100 if total_tests > 0 else 0
        results["accuracy_percentage"] = accuracy_percentage
        
        # Check if accuracy meets the >90% requirement
        results["meets_accuracy_requirement"] = accuracy_percentage >= 90.0
        
        print("\n--- RAG Accuracy Test Summary ---")
        print(f"Total Tests: {total_tests}")
        print(f"Correct Answers: {correct_answers}")
        print(f"Accuracy: {accuracy_percentage:.2f}%")
        print(f"Requirement (>90%): {'✓ PASSED' if results['meets_accuracy_requirement'] else '✗ FAILED'}")
        
        return results
    
    def _check_response_accuracy(self, response: str, expected_keywords: List[str]) -> bool:
        """
        Evaluates if the response is accurate by checking for the presence of expected keywords.
        A response is considered correct if it contains at least 70% of the expected keywords.
        """
        if not response:
            return False
            
        response_lower = response.lower()
        present_keywords_count = sum(1 for keyword in expected_keywords if keyword.lower() in response_lower)
        
        # Require at least 70% of keywords to be present for the answer to be considered correct
        required_keywords_count = int(len(expected_keywords) * 0.7)
        
        return present_keywords_count >= required_keywords_count

async def run_rag_accuracy_tests() -> bool:
    """
    Initializes and runs the RAG accuracy test suite.
    """
    print("Initializing RAG Accuracy Test...")
    tester = RAGAccuracyTester(rag_agent_instance)
    results = await tester.run_accuracy_test()
    
    # Save detailed results to a file for analysis
    with open("rag_accuracy_results.json", "w") as f:
        json.dump(results, f, indent=4)
        
    print("\nDetailed test results saved to 'rag_accuracy_results.json'")
    
    return results["meets_accuracy_requirement"]

if __name__ == "__main__":
    # This allows the test to be run directly.
    # It requires the environment variables (e.g., QDRANT_URL, OPENAI_API_KEY) to be set.
    print("Running RAG accuracy test directly...")
    
    # Ensure the event loop is managed correctly
    try:
        asyncio.run(run_rag_accuracy_tests())
    except RuntimeError as e:
        # In some environments (like Jupyter), an event loop is already running.
        if "cannot run loop while another loop is running" in str(e):
            loop = asyncio.get_running_loop()
            task = loop.create_task(run_rag_accuracy_tests())
        else:
            raise
    
    print("\nTest execution finished.")