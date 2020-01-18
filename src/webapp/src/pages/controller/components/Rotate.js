import React, { Component } from 'react'
import TextField from '@material-ui/core/TextField';
import Button from '@material-ui/core/Button';

export default class Rotate extends Component {
  state = {
    time: 0,
    destination: 0,
    currentPosition: 0,
    frequency: 0,
    inProgress: 'Unknow',
    commandQueue: [],
  }


  async handleSubmit(event, commandQueue) {
    event.preventDefault();

    fetch('http://localhost:5000/rotate', {
      method: 'POST',
      headers: {
        'Accept': 'application/json',
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(commandQueue)
    });
  }

  handleCommandQueueAdd(time, destination, currentPosition) {
    const commandQueue = this.state.commandQueue;
    const data = { time, destination, currentPosition }
    commandQueue.push(data)

    this.setState({
      commandQueue,
      time: 0,
      destination: 0,
      currentPosition: 0,
    })
  }

  render() {
    const { time, destination, currentPosition, commandQueue } = this.state;
    console.log(commandQueue)
    return (
      <div>
        <h1>Rotate</h1>
        <h4>Command queue</h4>

        {commandQueue.map((command, index) => (
          <div key={index}>
            <div>Time: {command.time}; Destination: {command.destination}; Current position {command.currentPosition}</div>
          </div>
        ))}

        <form className="controller-form" onSubmit={(event) => this.handleSubmit(event, commandQueue)}>
          <TextField
            id="standard-name"
            className="controller-form-input"
            label="Time"
            value={time}
            onChange={(event) => this.setState({ time: event.target.value })}
            margin="normal"
          />
          <TextField
            id="standard-name"
            className="controller-form-input"
            label="Destination"
            value={destination}
            onChange={(event) => this.setState({ destination: event.target.value })}
            margin="normal"
          />
          <TextField
            id="standard-name"
            className="controller-form-input"
            label="Current position"
            value={currentPosition}
            onChange={(event) => this.setState({ currentPosition: event.target.value })}
            margin="normal"
          />
          <Button className="register-submit-button" variant="contained" type="button"
            onClick={() => this.handleCommandQueueAdd(time, destination, currentPosition)}>Add</Button>
          <Button className="register-submit-button" variant="contained" type="submit">Submit</Button>
          <Button className="register-submit-button" variant="contained" type="button"
            onClick={() => this.setState({
              commandQueue: [],
              time: 0,
              destination: 0,
              currentPosition: 0,
            })}>Clear</Button>
        </form>
      </div>
    )
  }
}
