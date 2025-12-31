# ADR-0007: Urdu Translation Feature Removal

- **Status:** Superseded
- **Date:** 2025-12-30
- **Feature:** textbook-physical-ai
- **Context:** To maintain 100% free operation of the project, the Urdu translation feature has been removed. This decision was made to ensure the project exclusively uses free-tier services and remains accessible without any paid features or services.

## Decision

The **Urdu translation feature has been removed** from the project. The project will no longer include on-demand translation of textbook chapters. This ensures that the project remains 100% free to use and operates exclusively within free-tier service constraints.

## Consequences

### Positive

- **100% Free Operation:** The project now operates exclusively using free-tier services with no paid features.
- **Reduced Complexity:** Removing the translation feature simplifies the architecture and reduces potential points of failure.
- **Lower Resource Usage:** No additional computational resources are required for translation tasks.
- **Faster Development:** Development effort can be focused on core RAG chatbot functionality rather than translation features.

### Negative

- **Reduced Accessibility:** Non-English speakers may have reduced access to the content.
- **Limited Localization:** The project does not support multilingual access.

## Alternatives Considered

- **Keep Translation Feature:** Continue with the original plan to implement translation.
    - **Why Rejected:** This would require paid services or introduce complexity that conflicts with the project's goal of 100% free operation.

- **Community Translation:** Allow community contributions for translations.
    - **Why Rejected:** This would still require infrastructure and management overhead that conflicts with the free operation goal.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
