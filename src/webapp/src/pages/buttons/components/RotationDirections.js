import React, { Component } from 'react'
import TextField from '@material-ui/core/TextField';
import Button from '@material-ui/core/Button';

export default class RotationDirections extends Component {
  state = {
    horizontalLeft: false,
    horizontalRight: false,
    verticalUp: false,
    verticalDown: false,
  }


  async handleRequest(rotateOn, rotateId) {
    console.log(rotateOn, rotateId);
    fetch('http://localhost:5000/', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({action: rotateOn, motor: rotateId})
    });
  }

  handleRotation(rotateOn, rotateId) {
    switch(rotateId) {
        case 0:
            this.setState({
                horizontalLeft: !rotateOn,
                });
            break;
        case 1:
            this.setState({
                horizontalRight: !rotateOn,
                });
            break;
        case 2:
            this.setState({
                verticalUp: !rotateOn,
                });
            break;
        case 3:
            this.setState({
                verticalDown: !rotateOn,
                });
            break;
    }
    this.handleRequest(!rotateOn, rotateId);
  }
  

  render() {
    const { horizontalLeft, horizontalRight, verticalUp, verticalDown } = this.state;
    return (
      <div>
        <h1>Rotate</h1>
        <h4>Select direction and speed</h4>

        <Button className="motor-horizontal-left" onClick={() => this.handleRotation(this.state.horizontalLeft, 0)}>Left</Button>
        <Button className="motor-horizontal-right" onClick={() => this.handleRotation(this.state.horizontalRight, 1)}>Right</Button>
        <Button className="motor-vertical-up" onClick={() => this.handleRotation(this.state.verticalUp, 2)}>Up</Button>
        <Button className="motor-vertical-down" onClick={() => this.handleRotation(this.state.verticalDown, 3)}>Down</Button>

      </div>
    )
  }
}
