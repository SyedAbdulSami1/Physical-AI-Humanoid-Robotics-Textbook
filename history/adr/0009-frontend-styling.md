# ADR-0009: Frontend Styling

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** textbook-physical-ai
- **Context:** The Docusaurus frontend requires a styling solution to customize its appearance and ensure a consistent, modern look and feel for the textbook and its interactive components. The choice needs to balance rapid development, maintainability, and customization capabilities.

## Decision

We will use **TailwindCSS** for styling the Docusaurus site. Docusaurus has good support for integrating Tailwind, and it allows for rapid development through its utility-first approach. We will use the standard Docusaurus theme as a base and apply Tailwind utilities for custom components (like the chatbot) and for overriding or extending base styles where needed.

## Consequences

### Positive

- **Rapid Development:** The utility-first nature of Tailwind allows for styling directly in the markup, which is very fast for building custom components.
- **Consistency:** It's easy to enforce a consistent design system (colors, spacing, fonts) by configuring the `tailwind.config.js` file.
- **Performant:** When combined with `purgecss` (which is standard in Docusaurus Tailwind setups), the final CSS bundle will be very small, as unused styles are removed.
- **No Context Switching:** Developers can style components without switching between JSX and separate CSS files.

### Negative

- **Verbose Markup:** HTML/JSX can become cluttered with many utility classes, which can be a downside for readability if not managed well.
- **Learning Curve:** For developers not familiar with Tailwind, there is a learning curve to understand the utility classes.
- **Potential for Inconsistency:** If not governed by a strict configuration, the overuse of "magic numbers" or one-off values in the markup can lead to inconsistencies.

## Alternatives Considered

- **CSS Modules:** This is the default styling method in Docusaurus. It provides locally-scoped CSS, which is great for component encapsulation.
    - **Why Rejected:** While good, CSS Modules can be more verbose for rapid prototyping. It requires creating separate `.css` files for each component, which can slow down development compared to Tailwind's utility-first approach. We will still use it for the base Docusaurus styling, but custom components will use Tailwind.
- **Emotion / Styled-components (CSS-in-JS):** These libraries allow writing CSS directly within JavaScript components.
    - **Why Rejected:** While powerful, CSS-in-JS can add a runtime overhead and potentially increase the complexity of the React application. Given that Docusaurus is a static-first framework, a solution that generates static CSS at build time (like Tailwind) is a more natural fit.

## References

- Feature Spec: `specs/textbook-physical-ai/spec.md`
- Implementation Plan: `specs/textbook-physical-ai/plan.md`
- Related ADRs: (none)
- Evaluator Evidence: (none)
