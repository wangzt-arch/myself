import React from "react";
import "./index.scss";

function Loading(props) {
  const { show } = props
  return (
    show && <div className="loader-container">
      <div className="loader-child"></div>
      <div className="loader-child"></div>
      <div className="loader-child"></div>
    </div>
  );
}

export default Loading;
