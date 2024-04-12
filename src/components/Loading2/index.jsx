import React from "react";
import "./index.scss";

function Loading(props) {
  const { show } = props
  return (
    show && <div className="load-container2">
      <div className="container">
        <div className="boxLoading boxLoading1"></div>
        <div className="boxLoading boxLoading2"></div>
        <div className="boxLoading boxLoading3"></div>
        <div className="boxLoading boxLoading4"></div>
        <div className="boxLoading boxLoading5"></div>
      </div>
    </div>
  );
}

export default Loading;
