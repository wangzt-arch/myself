import React from "react";
import "./index.scss";

function Loading(props) {
  const { show } = props
  return (
    show && <div className="loading">加载中</div>
  );
}

export default Loading;
