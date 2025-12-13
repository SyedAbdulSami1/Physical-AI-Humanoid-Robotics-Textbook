import asyncio
import json
from typing import List, Dict, Tuple
from app.skills.rag_agent import RAGAgent
from qdrant_client import QdrantClient
import openai
import os
from sqlalchemy.orm import Session

class RAGAccuracyTester:
    """
    Tests the accuracy of the RAG system to ensure >90% accuracy
    """
    
    def __init__(self, rag_agent: RAGAgent, db: Session):
        self.rag_agent = rag_agent
        self.db = db
        self.test_cases = self._load_test_cases()
    
    def _load_test_cases(self) -> List[Dict]:
        """
        Load test cases for accuracy testing
        In a real implementation, these would be loaded from a test dataset
        """
        return [
            {
                "query": "What is forward kinematics in robotics?",
                "expected_keywords": ["kinematics", "position", "motion", "joint angles", "end-effector"],
                "context": "Forward kinematics is the use of kinematic equations to determine the position and orientation of the end-effector based on the joint parameters of the robot."
            },
            {
                "query": "Explain PID controller in humanoid robots",
                "expected_keywords": ["PID", "proportional", "integral", "derivative", "control", "error"],
                "context": "A PID controller in humanoid robots is a control loop mechanism that uses proportional, integral, and derivative terms to minimize error in the system."
            },
            {
                "query": "What is sensor fusion in robotics?",
                "expected_keywords": ["sensor fusion", "multiple sensors", "data integration", "accuracy", "reliability"],
                "context": "Sensor fusion is the process of combining data from multiple sensors to achieve more accurate and reliable information than what could be obtained from any single sensor."
            }
        ]
    
    async def run_accuracy_test(self) -> Dict:
        """
        Run the accuracy test and return results
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
            print(f"Running test {i+1}/{total_tests}: {test_case['query']}")
            
            # Instead of using search results from Qdrant, we'll simulate with context
            # In a real scenario, the RAG agent would search and find relevant content
            search_results_mock = [{"payload": {"text": test_case["context"]}}]
            
            try:
                # Get response from RAG agent
                response = await self.rag_agent.generate_response(
                    query=test_case["query"],
                    search_results=search_results_mock
                )
                
                # Check if response contains expected keywords
                is_correct = self._check_response_accuracy(response, test_case["expected_keywords"])
                
                result_detail = {
                    "query": test_case["query"],
                    "response": response,
                    "expected_keywords": test_case["expected_keywords"],
                    "is_correct": is_correct
                }
                
                results["detailed_results"].append(result_detail)
                
                if is_correct:
                    correct_answers += 1
                    results["correct_answers"] += 1
                
                print(f"  Result: {'✓ PASS' if is_correct else '✗ FAIL'}")
                
            except Exception as e:
                print(f"  Error in test {i+1}: {str(e)}")
                results["detailed_results"].append({
                    "query": test_case["query"],
                    "response": None,
                    "expected_keywords": test_case["expected_keywords"],
                    "is_correct": False,
                    "error": str(e)
                })
        
        # Calculate accuracy
        accuracy_percentage = (correct_answers / total_tests) * 100
        results["accuracy_percentage"] = accuracy_percentage
        
        # Check if accuracy meets the >90% requirement
        results["meets_accuracy_requirement"] = accuracy_percentage > 90.0
        
        print(f"\nTest Results:")
        print(f"Correct Answers: {correct_answers}/{total_tests}")
        print(f"Accuracy: {accuracy_percentage:.2f}%")
        print(f"Meets >90% requirement: {'YES' if results['meets_accuracy_requirement'] else 'NO'}")
        
        return results
    
    def _check_response_accuracy(self, response: str, expected_keywords: List[str]) -> bool:
        """
        Check if the response contains expected keywords
        This is a basic implementation - in production, more sophisticated NLP techniques would be used
        """
        response_lower = response.lower()
        
        # Count how many expected keywords are present in the response
        present_keywords = [keyword for keyword in expected_keywords if keyword.lower() in response_lower]
        
        # Require at least 70% of expected keywords to be present for accuracy
        required_keywords = max(1, int(len(expected_keywords) * 0.7))  # At least 70% of keywords
        
        return len(present_keywords) >= required_keywords

# Function to run RAG accuracy tests
async def run_rag_accuracy_tests(rag_agent: RAGAgent, db: Session) -> bool:
    """
    Run RAG accuracy tests to ensure >90% accuracy
    """
    tester = RAGAccuracyTester(rag_agent, db)
    results = await tester.run_accuracy_test()
    
    if results["meets_accuracy_requirement"]:
        print("\n✓ RAG accuracy test PASSED! Accuracy is above 90%")
        return True
    else:
        print(f"\n✗ RAG accuracy test FAILED! Accuracy is {results['accuracy_percentage']:.2f}% (requirement: >90%)")
        return False

# Example usage
if __name__ == "__main__":
    # This is just an example of how the testing would work
    # In practice, you would initialize the RAG agent with proper dependencies
    print("RAG accuracy testing module. Run from the main application context.")