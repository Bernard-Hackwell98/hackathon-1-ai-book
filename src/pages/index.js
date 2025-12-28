import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <div className="container">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <h1 className="hero__title">{siteConfig.title}</h1>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/modules/ros2-humanoid/intro">
              ROS 2 Module
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/modules/physical-ai/intro">
              Physical AI Module
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}