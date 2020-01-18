import React, { Component } from 'react';
import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import RegisterItems from './components/RegisterItems';

class Registers extends Component {

    constructor(props) {
        super(props);

        this.state = {
            coils: [],
            registers: [],
            value: 0,
        }
    }

    async retriveCoils() {
        const response = await fetch('http://localhost:5000/coil');
        const data = await response.json()
        return data;
    }

    async retriveRegisters() {
        const response = await fetch('http://localhost:5000/register');
        const data = await response.json()
        return data;
    }

    async componentDidMount() {
        const coils = await this.retriveCoils();
        const registers = await this.retriveRegisters();

        this.setState({ coils, registers });
    }

    render() {
        const { coils, registers, value } = this.state;

        return (
            <div>
                <AppBar position="static">
                    <Tabs value={value} onChange={(event, newValue) => this.setState({ value: newValue })}>
                        <Tab label="All" />
                        <Tab label="Coils" />
                        <Tab label="Registers" />
                        <Tab label="Controller" onClick={() => this.props.history.push('/controller')} />
                    </Tabs>
                </AppBar>
                <div className="registers">
                    {value === 0 || value === 1 ? <RegisterItems items={coils} header={'Coils'} type="coil" /> : ''}
                    {value === 0 || value === 2 ? <RegisterItems items={registers} header={'Registers'} type="register" /> : ''}
                </div>
            </div>
        )
    }
}

export default Registers;