import React from "react";
import "./index.scss";

function Loading(props) {
  const { show } = props
  return (
    show && <div class="loader-container">
      <div class="loader-child"></div>
      <div class="loader-child"></div>
      <div class="loader-child"></div>
    </div>
  );
}

export default Loading;
