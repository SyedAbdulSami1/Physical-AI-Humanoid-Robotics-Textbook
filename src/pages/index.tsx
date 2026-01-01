import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// --- Main Header Component ---
function HomepageHeader() {
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className={styles.heroTitle}>
          Physical <span className={styles.highlight}>AI & Humanoid</span> Robotics
        </Heading>
        <p className={styles.heroSubtitle}>
          A comprehensive, AI-native textbook designed to bridge the gap between artificial intelligence and the physical world. This course focuses on embodied intelligence—AI systems that operate in reality—enabling you to control humanoid robots in simulated and real-world environments.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

// --- Course Highlights Section ---
function CourseContentSection() {
  return (
    <section className={styles.contentSection}>
      <div className="container">
        <Heading as="h2" className={clsx('text--center', styles.sectionTitle)}>What You'll Learn</Heading>
        <div className={styles.featuresGrid}>
          <div className={styles.featureCard}>
            <h3 className={styles.featureTitle}>Robotic Nervous System</h3>
            <p>Master the fundamentals of ROS 2, the open-source framework for building robust robot applications.</p>
          </div>
          <div className={styles.featureCard}>
            <h3 className={styles.featureTitle}>Digital Twin Simulation</h3>
            <p>Learn to build and control robots in high-fidelity simulations using Gazebo and Unity.</p>
          </div>
          <div className={styles.featureCard}>
            <h3 className={styles.featureTitle}>AI-Robot Brain</h3>
            <p>Explore advanced perception and navigation using powerful tools like NVIDIA Isaac™.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

// --- Testimonials Section ---
function TestimonialsSection() {
  return (
    <section className={clsx(styles.contentSection, styles.testimonialsSection)}>
      <div className="container">
        <Heading as="h2" className={clsx('text--center', styles.sectionTitle)}>Praised by Students & Professionals</Heading>
        <div className={styles.testimonialsGrid}>
          <div className={styles.testimonialCard}>
            <blockquote>
              “This textbook is a game-changer. It connects AI theory with practical robotics in a way I haven't seen anywhere else. Essential for any aspiring roboticist.”
            </blockquote>
            <cite>— Alex R., Computer Science Student</cite>
          </div>
          <div className={styles.testimonialCard}>
            <blockquote>
              “As a hobbyist, the world of humanoid robotics seemed inaccessible. This course broke it down perfectly. The simulation modules are fantastic.”
            </blockquote>
            <cite>— Sam K., Robotics Hobbyist</cite>
          </div>
        </div>
      </div>
    </section>
  );
}

// --- Call to Action Section ---
function CtaSection() {
  return (
    <section className={clsx(styles.contentSection, styles.ctaSection, 'text--center')}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>Start Your Journey Today</Heading>
        <p className={styles.ctaSubtitle}>
          Dive into the world of embodied intelligence and begin building the next generation of humanoid robots.
        </p>
        <Link className="button button--primary button--lg" to="/docs/intro">
          Begin the Course
        </Link>
      </div>
    </section>
  );
}


// --- Main Home Component ---
export default function Home(): React.ReactElement {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Welcome"
      description="An AI-native textbook for embodied intelligence, combining robotics and AI.">
      <HomepageHeader />
      <main>
        <CourseContentSection />
        <TestimonialsSection />
        <CtaSection />
      </main>
    </Layout>
  );
}
