# ADR-0007: Urdu Translation Implementation

- **Status:** Proposed
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** To enhance accessibility, the project includes a feature for on-demand Urdu translation of textbook chapters. The decision lies between using a third-party cloud-based translation service or a self-hosted open-source model. Key factors are cost, translation quality, and implementation complexity, especially within a hackathon's constraints.

## Decision

We will use a **local, open-source Hugging Face model** for translation. Specifically, we will start with a model from the NLLB (No Language Left Behind) family or a similar high-quality model optimized for English-to-Urdu translation. This model will be run on-demand within our FastAPI backend. This approach avoids external API costs and keeps the entire translation pipeline within our control.

## Consequences

### Positive

- **No Direct Cost:** There are no per-character or per-request costs associated with a self-hosted model, which is ideal for a free-to-use educational tool.
- **Data Privacy:** The content being translated never leaves our own backend server, which is a good practice even if the content is public.
- **Control & Customization:** We have full control over the model and could potentially fine-tune it on our specific domain if needed (though this is out of scope for the hackathon).

### Negative

- **Infrastructure Overhead:** We are responsible for downloading, managing, and running the model. This will increase the resource requirements (RAM and potentially GPU) of our backend server.
- **Potentially Lower Quality:** While modern open-source models are very good, they may not match the quality of large-scale commercial services like Google Translate or DeepL for all types of content.
- **Performance Impact:** Running the translation model on the same server as the API can impact the performance and responsiveness of other endpoints. The first translation request might be slow as the model is loaded into memory.

## Alternatives Considered

- **Paid Cloud Translation API (e.g., Google Translate, DeepL):** Using a managed API for translation.
    - **Why Rejected:** These services are excellent but incur costs based on usage. For a hackathon project aiming for a free or low-cost stack, integrating a paid API for a core feature is not ideal. It would also introduce another external dependency and API key to manage.

- **Batch Pre-translation:** Translating all content beforehand and serving the Urdu version as static files.
    - **Why Rejected:** This would double the amount of content to manage and store. It also loses the "on-demand" nature of the feature, making it less dynamic. Any update to the English content would require a full re-translation of everything. The on-demand approach is more flexible.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
