import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useColorMode } from '@docusaurus/theme-common';

const ImageByColorMode = ({lightModeImage, darkModeImage}) => {
  const { colorMode } = useColorMode();

  return (
    <img
      src={colorMode === 'dark' ? darkModeImage : lightModeImage}
      alt={colorMode === 'dark' ? darkModeImage : lightModeImage}
      style={{ width: '100%', height: 'auto' }}
    />
  );
};

export default ImageByColorMode;
