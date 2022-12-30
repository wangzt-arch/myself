import React from "react";
import "./index.scss";

function Loading(props) {
  const { show } = props
  return (
    show && <div class="load-container2">
      <div class="container">
        <div class="boxLoading boxLoading1"></div>
        <div class="boxLoading boxLoading2"></div>
        <div class="boxLoading boxLoading3"></div>
        <div class="boxLoading boxLoading4"></div>
        <div class="boxLoading boxLoading5"></div>
      </div>
    </div>
  );
}

export default Loading;
