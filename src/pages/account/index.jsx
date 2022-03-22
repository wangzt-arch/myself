import React, { Component } from "react";
import { connect } from "react-redux";
import Home from "../home/index";
import { incrementAction, reduceAction } from "../../reducers/calculate";
import "./index.css";

const mapStateToProps = (state) => {
  return {
    num: state.calculate.num,
  };
};

const mapDispatchToProps = (dispatch) => ({
  increment: () => dispatch(incrementAction),
  decrement: () => dispatch(reduceAction),
});

class Account extends Component {
  render() {
    return (
      <>
        <Home></Home>
        <div className="container">
          <p onClick={this.props.increment}>click to increment num</p>
          <p onClick={this.props.decrement}>click to decrement num</p>
          <p>{this.props.num}</p>
        </div>
      </>
    );
  }
}
export default connect(mapStateToProps, mapDispatchToProps)(Account);
