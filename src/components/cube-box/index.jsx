import React from "react";
import react from "./image/react.svg";
import sass from "./image/sass.svg";
import ts from "./image/ts.svg";
import vue from "./image/vue.svg";
import vscode from "./image/vscode.svg";

import "./index.scss";

function CubeBox() {
  return (
    <div className="cube-box">
      <div className="cube-box_item cube-box__right">
        <img width="100%" height="100%" src={react} alt="react"></img>
      </div>
      <div className="cube-box_item cube-box__left">
        <img width="100%" height="100%" src={sass} alt="react"></img>
      </div>
      <div className="cube-box_item cube-box__top">
        <img width="100%" height="100%" src={ts} alt="react"></img>
      </div>
      <div className="cube-box_item cube-box__bottom">
        <img width="100%" height="100%" src={vue} alt="react"></img>
      </div>
      <div className="cube-box_item cube-box__center">
        <img width="100%" height="100%" src={vscode} alt="react"></img>
      </div>
    </div>
  );
}

export default CubeBox;
