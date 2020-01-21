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
    fetch('http://localhost:5000/buttonrotate', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body: JSON.stringify([rotateOn, rotateId])
    });
  }

  handleRotation(rotateOn, rotateId) {
    let allowRotate = false;
    switch(rotateId) {
        case 0:
          if(!(this.state.horizontalRight || this.state.verticalUp || this.state.verticalDown)){
            allowRotate = true;
            this.setState({
              horizontalLeft: !rotateOn,
            });
          }
          break;
        case 1:
          if(!(this.state.horizontalLeft || this.state.verticalDown || this.state.verticalUp)){
            allowRotate = true;
            this.setState({
              horizontalRight: !rotateOn,
            });
          }
          break;
        case 2:
          if(!(this.state.verticalDown || this.state.horizontalRight || this.state.horizontalLeft)){
            allowRotate = true;
            this.setState({
              verticalUp: !rotateOn,
            });
          }
          break;
        case 3:
          if(!(this.state.verticalUp || this.state.horizontalRight || this.state.horizontalLeft)){
            allowRotate = true;
            this.setState({
              verticalDown: !rotateOn,
            });
          }
          break;

    }
    if(allowRotate){
      this.handleRequest(!rotateOn, rotateId);
    }
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
