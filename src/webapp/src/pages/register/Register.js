import React, { Component } from 'react';
import { withRouter } from 'react-router-dom';
import Modal from '@material-ui/core/Modal';
import Paper from '@material-ui/core/Paper';
import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableRow from '@material-ui/core/TableRow';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';

class Register extends Component {

    constructor(props) {
        super(props);
        this.state = {
            register: {
                description: "",
                example: "",
                function_code: "",
                hex_address: "",
                name: "Unknow",
                port: "",
                slave: -1,
                write_access: false
            },
            currentValue: -1,
            inputValue: '',
            response: {
                address: 'Unknown',
                decimal_address: -1,
                value: -1,
                status: "Unknown"
            },
        }
    }

    async handleSubmit(event, currentNode) {
        event.preventDefault();
        const { props, state } = currentNode

        const type = props.match.params.type;
        const name = props.match.params.name;

        const response = await fetch('http://localhost:5000/' + type + '/' + name, {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ value: state.inputValue })
        });

        const data = await response.json();
        const inverterData = await this.retriveInverterData()

        this.setState({ response: data, currentValue: inverterData });
    }

    async retriveInverterData() {
        const { match } = this.props;
        const type = match.params.type;
        const name = match.params.name;

        const response = await fetch('http://localhost:5000/' + type + '/' + name + '/value');
        const data = await response.json()
        return data;
    }

    async retriveData() {
        const { match } = this.props;
        const type = match.params.type;
        const name = match.params.name;

        const response = await fetch('http://localhost:5000/' + type + '/' + name);
        const data = await response.json()
        return data;
    }

    async componentDidMount() {
        const data = await this.retriveData();
        const inverterData = await this.retriveInverterData()
        this.setState({ register: data, currentValue: inverterData });
    }

    render() {
        const { name, description, example, port, slave, write_access, is_coil } = this.state.register
        const { address, decimal_address, value, status } = this.state.response;
        const coilValue = !this.state.currentValue ? '0' : '1';
        const registerValue = this.state.currentValue;

        const endValue = is_coil ? coilValue : registerValue;

        return (
            <Modal open={true} onClose={() => this.props.history.goBack()}>
                <Paper className="register-view" elevation={1}>
                    <h1>{name}</h1>
                    <Table>
                        <TableBody>
                            <TableRow>
                                <TableCell component="th" scope="row" className="register-item-key">
                                    Description
                                </TableCell>
                                <TableCell align="left">{description}</TableCell>
                            </TableRow>
                            <TableRow>
                                <TableCell component="th" scope="row" className="register-item-key">
                                    Example
                                </TableCell>
                                <TableCell align="left">{example}</TableCell>
                            </TableRow>
                            <TableRow>
                                <TableCell component="th" scope="row" className="register-item-key">
                                    Port
                                </TableCell>
                                <TableCell align="left">{port}</TableCell>
                            </TableRow>
                            <TableRow>
                                <TableCell component="th" scope="row" className="register-item-key">
                                    Slave
                                </TableCell>
                                <TableCell align="left">{slave}</TableCell>
                            </TableRow>
                            <TableRow>
                                <TableCell component="th" scope="row" className="register-item-key">
                                    Write access
                                </TableCell>
                                <TableCell align="left">{write_access ? 'R/W' : 'R'}</TableCell>
                            </TableRow>
                            <TableRow>
                                <TableCell component="th" scope="row" className="register-item-key">
                                    Current value
                                </TableCell>
                                <TableCell align="left">{endValue}</TableCell>
                            </TableRow>
                        </TableBody>
                    </Table>
                    {write_access ? (<div>
                        <h1>Write to inverter</h1>
                        <form className="form" onSubmit={(event) => this.handleSubmit(event, this)}>
                            <TextField
                                id="standard-name"
                                label="Value"
                                value={this.state.inputValue}
                                onChange={(event) => this.setState({ inputValue: event.target.value })}
                                margin="normal"
                            />
                            <Button className="register-submit-button" variant="contained" type="submit">Submit</Button>
                        </form>
                    </div>) : ''}

                    {write_access ? (<div>
                        <h1>Response</h1>
                        <Table>
                            <TableBody>
                                <TableRow>
                                    <TableCell component="th" scope="row" className="register-item-key">
                                        Address
                                </TableCell>
                                    <TableCell align="left">{address}</TableCell>
                                </TableRow>
                                <TableRow>
                                    <TableCell component="th" scope="row" className="register-item-key">
                                        Decimal
                                </TableCell>
                                    <TableCell align="left">{decimal_address}</TableCell>
                                </TableRow>
                                <TableRow>
                                    <TableCell component="th" scope="row" className="register-item-key">
                                        Value
                                </TableCell>
                                    <TableCell align="left">{value}</TableCell>
                                </TableRow>
                                <TableRow>
                                    <TableCell component="th" scope="row" className="register-item-key">
                                        Status
                                </TableCell>
                                    <TableCell align="left">{status}</TableCell>
                                </TableRow>
                            </TableBody>
                        </Table>
                    </div>) : ''}

                </Paper>
            </Modal>
        )
    }
}

export default withRouter(Register);