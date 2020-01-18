import React, { Component } from 'react'

export default class Settings extends Component {

  state = {
    port: '',
    slave: '',
  }

  async retriveSettings() {
    const response = await fetch('http://localhost:5000/setting');
    const data = await response.json()
    return data;
  }

  async componentDidMount() {
    const settings = await this.retriveSettings();
    this.setState({ port: settings.port, slave: settings.slave });
  }

  render() {
    return (
      <div>
        <h1>Settings</h1>
        <h4>Port: {this.state.port}</h4>
        <h4>Slave: {this.state.slave}</h4>
      </div>
    )
  }
}
