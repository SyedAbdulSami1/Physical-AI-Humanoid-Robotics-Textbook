# ADR-0005: RAG Content Chunking Strategy

- **Status:** Proposed
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The effectiveness of the Retrieval-Augmented Generation (RAG) chatbot is highly dependent on the quality of the content chunks retrieved from the vector database. The chunking strategy determines how the Markdown documents are split before being embedded. A poor strategy can lead to fragmented, out-of-context, or irrelevant information being passed to the LLM, resulting in poor quality answers.

## Decision

We will implement a **Semantic Chunking** strategy using a library like `langchain` or a similar text-splitter that can handle Markdown. Specifically, we will aim for a `RecursiveCharacterTextSplitter` configured to prioritize splitting along Markdown structural elements (paragraphs, headers, code blocks) before falling back to character-based splitting. This creates chunks that are more likely to be semantically coherent. We will aim for a chunk size of around 1000 characters with an overlap of 200 characters to maintain context between chunks.

## Consequences

### Positive

- **Improved Retrieval Accuracy:** Chunks are more likely to contain complete thoughts or code blocks, leading to more relevant search results from the vector store.
- **Better LLM Performance:** The LLM will receive more coherent and contextually complete information, enabling it to generate higher quality and more accurate answers.
- **Markdown-Aware:** The strategy respects the structure of the source documents, preventing code blocks or tables from being awkwardly split.

### Negative

- **Complexity:** Implementing a semantic chunking strategy is more complex than simple fixed-size chunking. It requires careful configuration and testing.
- **Processing Overhead:** The initial processing of documents to create semantic chunks can be more computationally intensive than naive splitting.

## Alternatives Considered

- **Fixed-size Chunking:** Splitting documents into chunks of a fixed number of characters.
    - **Why Rejected:** This is the simplest method but also the most brittle. It can easily split a sentence, code block, or thought in half, leading to fragmented and out-of-context chunks that will degrade the performance of the RAG system.

- **Recursive Chunking (Character-based only):** A simple recursive text splitter without Markdown-specific separators.
    - **Why Rejected:** While better than fixed-size, it might still not optimally handle the structure of Markdown files, potentially splitting code blocks or tables in ways that a Markdown-aware splitter would avoid.

- **Agent-based Chunking:** Using an LLM to intelligently divide the text into logical sections.
    - **Why Rejected:** This approach is very powerful but also slow and expensive, especially for a large corpus. It's overkill for this project's requirements and budget.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
