---
id: 1
title: Amend Constitution RAG Standards
stage: constitution
date_iso: 2025-12-15
surface: agent
model: Qwen
feature: rag-chatbot
branch: main
user: user
command: /sp.constitution
labels:
  - constitution
  - rag
  - standards
links:
  - spec: null
  - ticket: null
  - adr: null
  - pr: null
files_yaml:
  - ".specify/memory/constitution.md"
tests_yaml: []
---

# Amendment to Project Constitution for RAG Chatbot Implementation

## Summary
Added Articles 8.1 through 8.14 to the project constitution, establishing mandatory standards for the RAG chatbot implementation. These new articles cover all aspects of the chatbot development including technology stack requirements, implementation standards, and quality gates.

## Changes Made
- Preserved all existing constitution content (Articles 1-7)
- Added 14 new articles (Article 8) with specific requirements for:
  - FastAPI backend framework usage with async endpoints
  - Neon Serverless Postgres for user/session storage
  - Qdrant Cloud Free Tier for vector embeddings
  - OpenAI/ChatKit SDKs for generation
  - Selected-text-only RAG mode implementation
  - Better-Auth integration with background questionnaire
  - Personalization logic using stored user profiles
  - Urdu translation capability with reliable service
  - Automated ingestion pipeline for book Markdown content
  - Authentication middleware requirements
  - Error handling and resilience measures
  - RAG accuracy testing protocols
  - Skills/subagents architecture for chatbot workflows

## Technical Requirements Included
Each new article follows the format of Standard, Rationale, and Quality Gates to ensure implementation consistency and measurable outcomes.

## Motivation
This amendment ensures that the RAG chatbot implementation meets the same high-quality standards as the rest of the Physical AI & Humanoid Robotics textbook project while providing clear guidelines for developers contributing to the chatbot functionality.