import React from 'react';
import Footer from '../components/Footer/index';
import Header from '../components/Header/index';
import '../styles/base.css';

function App({ Component, pageProps }) {
  return (
    <div className="container">
      <Header></Header>
      <Component {...pageProps} />
      {/* <Footer></Footer> */}
    </div>
  );
};

export default App;
