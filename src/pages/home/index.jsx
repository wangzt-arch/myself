import React, { Component } from "react";
import { Link } from "react-router-dom";
import "./index.css";

export default class Home extends Component {
  goto = (e) => {
    this.props.history.push(`/${e}`);
  };
  render() {
    return (
      <div className="home">
         home
      </div>
    );
  }
}
