import React from 'react';
import {useColorMode} from '@docusaurus/theme-common';
import {MdLightMode, MdDarkMode} from 'react-icons/md';
import styles from './ColorModeToggle.module.css';

export default function ColorModeToggle(): JSX.Element {
  const {colorMode, setColorMode} = useColorMode();

  const toggleColorMode = () => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  };

  return (
    <button
      className={styles.colorModeToggle}
      onClick={toggleColorMode}
      aria-label={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
      title={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
    >
      {colorMode === 'dark' ? (
        <MdLightMode className={styles.icon} />
      ) : (
        <MdDarkMode className={styles.icon} />
      )}
    </button>
  );
}
