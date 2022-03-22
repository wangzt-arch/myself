import React from 'react';
import Header from '../components/Header/index';
import '../styles/base.css';

function App({ Component, pageProps }) {
  return (
    <div className="container">
      <Header></Header>
      <Component {...pageProps} />
    </div>
  );
};

export default App;