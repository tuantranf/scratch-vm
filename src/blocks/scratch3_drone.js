const ArgumentType = require('../extension-support/argument-type');
const BlockType = require('../extension-support/block-type');
const color = require('../util/color');
const log = require('../util/log');

/**
 * Manage power, direction, and timers for one WeDo 2.0 drone.
 */
class Drone {
    /**
     * Construct a Drone instance.
     * @param {WeDo2} parent - the WeDo 2.0 device which owns this drone.
     * @param {int} index - the zero-based index of this drone on its parent device.
     */
    constructor (parent, index) {
        /**
         * The WeDo 2.0 device which owns this drone.
         * @type {WeDo2}
         * @private
         */
        this._parent = parent;

        /**
         * The zero-based index of this drone on its parent device.
         * @type {int}
         * @private
         */
        this._index = index;

        /**
         * This drone's current direction: 1 for "this way" or -1 for "that way"
         * @type {number}
         * @private
         */
        this._direction = 1;

        /**
         * This drone's current power level, in the range [0,100].
         * @type {number}
         * @private
         */
        this._power = 100;

        /**
         * Is this drone currently moving?
         * @type {boolean}
         * @private
         */
        this._isOn = false;

        /**
         * If the drone has been turned on or is actively braking for a specific duration, this is the timeout ID for
         * the end-of-action handler. Cancel this when changing plans.
         * @type {Object}
         * @private
         */
        this._pendingTimeoutId = null;

        this.startBraking = this.startBraking.bind(this);
        this.setDroneOff = this.setDroneOff.bind(this);
    }

    /**
     * @return {number} - the duration of active braking after a call to startBraking(). Afterward, turn the drone off.
     * @constructor
     */
    static get BRAKE_TIME_MS () {
        return 1000;
    }

    /**
     * @return {int} - this drone's current direction: 1 for "this way" or -1 for "that way"
     */
    get direction () {
        return this._direction;
    }

    /**
     * @param {int} value - this drone's new direction: 1 for "this way" or -1 for "that way"
     */
    set direction (value) {
        if (value < 0) {
            this._direction = -1;
        } else {
            this._direction = 1;
        }
    }

    /**
     * @return {int} - this drone's current power level, in the range [0,100].
     */
    get power () {
        return this._power;
    }

    /**
     * @param {int} value - this drone's new power level, in the range [0,100].
     */
    set power (value) {
        this._power = Math.max(0, Math.min(value, 100));
    }

    /**
     * @return {boolean} - true if this drone is currently moving, false if this drone is off or braking.
     */
    get isOn () {
        return this._isOn;
    }

    /**
     * Turn this drone on indefinitely.
     */
    setDroneOn () {
        this._parent._send('droneOn', {droneIndex: this._index, power: this._direction * this._power});
        this._isOn = true;
        this._clearTimeout();
    }

    /**
     * Turn this drone on for a specific duration.
     * @param {number} milliseconds - run the drone for this long.
     */
    setDroneOnFor (milliseconds) {
        milliseconds = Math.max(0, milliseconds);
        this.setDroneOn();
        this._setNewTimeout(this.startBraking, milliseconds);
    }

    /**
     * Start active braking on this drone. After a short time, the drone will turn off.
     */
    startBraking () {
        this._parent._send('droneBrake', {droneIndex: this._index});
        this._isOn = false;
        this._setNewTimeout(this.setDroneOff, Drone.BRAKE_TIME_MS);
    }

    /**
     * Turn this drone off.
     */
    setDroneOff () {
        this._parent._send('droneOff', {droneIndex: this._index});
        this._isOn = false;
    }

    /**
     * Clear the drone action timeout, if any. Safe to call even when there is no pending timeout.
     * @private
     */
    _clearTimeout () {
        if (this._pendingTimeoutId !== null) {
            clearTimeout(this._pendingTimeoutId);
            this._pendingTimeoutId = null;
        }
    }

    /**
     * Set a new drone action timeout, after clearing an existing one if necessary.
     * @param {Function} callback - to be called at the end of the timeout.
     * @param {int} delay - wait this many milliseconds before calling the callback.
     * @private
     */
    _setNewTimeout (callback, delay) {
        this._clearTimeout();
        const timeoutID = setTimeout(() => {
            if (this._pendingTimeoutId === timeoutID) {
                this._pendingTimeoutId = null;
            }
            callback();
        }, delay);
        this._pendingTimeoutId = timeoutID;
    }
}

/**
 * Manage communication with a WeDo 2.0 device over a Device Manager client socket.
 */
class WeDo2 {

    /**
     * @return {string} - the type of Device Manager device socket that this class will handle.
     */
    static get DEVICE_TYPE () {
        return 'wedo2';
    }

    /**
     * Construct a WeDo2 communication object.
     * @param {Socket} socket - the socket for a WeDo 2.0 device, as provided by a Device Manager client.
     */
    constructor (socket) {
        /**
         * The socket-IO socket used to communicate with the Device Manager about this device.
         * @type {Socket}
         * @private
         */
        this._socket = socket;

        /**
         * The drones which this WeDo 2.0 could possibly have.
         * @type {[Drone]}
         * @private
         */
        this._drones = [new Drone(this, 0), new Drone(this, 1)];

        /**
         * The most recently received value for each sensor.
         * @type {Object.<string, number>}
         * @private
         */
        this._sensors = {
            tiltX: 0,
            tiltY: 0,
            distance: 0
        };

        this._onSensorChanged = this._onSensorChanged.bind(this);
        this._onDisconnect = this._onDisconnect.bind(this);

        this._connectEvents();
    }

    /**
     * Manually dispose of this object.
     */
    dispose () {
        this._disconnectEvents();
    }

    /**
     * @return {number} - the latest value received for the tilt sensor's tilt about the X axis.
     */
    get tiltX () {
        return this._sensors.tiltX;
    }

    /**
     * @return {number} - the latest value received for the tilt sensor's tilt about the Y axis.
     */
    get tiltY () {
        return this._sensors.tiltY;
    }

    /**
     * @return {number} - the latest value received from the distance sensor.
     */
    get distance () {
        return this._sensors.distance * 10;
    }

    /**
     * Access a particular drone on this device.
     * @param {int} index - the zero-based index of the desired drone.
     * @return {Drone} - the Drone instance, if any, at that index.
     */
    drone (index) {
        return this._drones[index];
    }

    /**
     * Set the WeDo 2.0 hub's LED to a specific color.
     * @param {int} rgb - a 24-bit RGB color in 0xRRGGBB format.
     */
    setLED (rgb) {
        this._send('setLED', {rgb});
    }

    /**
     * Play a tone from the WeDo 2.0 hub for a specific amount of time.
     * @param {int} tone - the pitch of the tone, in Hz.
     * @param {int} milliseconds - the duration of the note, in milliseconds.
     */
    playTone (tone, milliseconds) {
        this._send('playTone', {tone, ms: milliseconds});
    }

    /**
     * Stop the tone playing from the WeDo 2.0 hub, if any.
     */
    stopTone () {
        this._send('stopTone');
    }

    /**
     * Attach event handlers to the device socket.
     * @private
     */
    _connectEvents () {
        this._socket.on('sensorChanged', this._onSensorChanged);
        this._socket.on('deviceWasClosed', this._onDisconnect);
        this._socket.on('disconnect', this._onDisconnect);
    }

    /**
     * Detach event handlers from the device socket.
     * @private
     */
    _disconnectEvents () {
        this._socket.off('sensorChanged', this._onSensorChanged);
        this._socket.off('deviceWasClosed', this._onDisconnect);
        this._socket.off('disconnect', this._onDisconnect);
    }

    /**
     * Store the sensor value from an incoming 'sensorChanged' event.
     * @param {object} event - the 'sensorChanged' event.
     * @property {string} sensorName - the name of the sensor which changed.
     * @property {number} sensorValue - the new value of the sensor.
     * @private
     */
    _onSensorChanged (event) {
        this._sensors[event.sensorName] = event.sensorValue;
    }

    /**
     * React to device disconnection. May be called more than once.
     * @private
     */
    _onDisconnect () {
        this._disconnectEvents();
    }

    /**
     * Send a message to the device socket.
     * @param {string} message - the name of the message, such as 'playTone'.
     * @param {object} [details] - optional additional details for the message, such as tone duration and pitch.
     * @private
     */
    _send (message, details) {
        this._socket.emit(message, details);
    }
}

/**
 * Enum for drone specification.
 * @readonly
 * @enum {string}
 */
const DroneID = {
    DEFAULT: 'drone',
    A: 'drone A',
    B: 'drone B',
    ALL: 'all drones'
};

/**
 * Enum for drone direction specification.
 * @readonly
 * @enum {string}
 */
const DroneDirection = {
    FORWARD: 'this way',
    BACKWARD: 'that way',
    REVERSE: 'reverse'
};

/**
 * Enum for tilt sensor direction.
 * @readonly
 * @enum {string}
 */
const TiltDirection = {
    UP: 'up',
    DOWN: 'down',
    LEFT: 'left',
    RIGHT: 'right',
    ANY: 'any'
};

/**
 * Scratch 3.0 blocks to interact with a LEGO WeDo 2.0 device.
 */
class Scratch3DroneBlocks {

    /**
     * @return {string} - the ID of this extension.
     */
    static get EXTENSION_ID () {
        return 'wedo2';
    }

    /**
     * @return {number} - the tilt sensor counts as "tilted" if its tilt angle meets or exceeds this threshold.
     */
    static get TILT_THRESHOLD () {
        return 15;
    }

    /**
     * Construct a set of WeDo 2.0 blocks.
     * @param {Runtime} runtime - the Scratch 3.0 runtime.
     */
    constructor (runtime) {
        /**
         * The Scratch 3.0 runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;

        this.connect();
    }

    /**
     * @returns {object} metadata for this extension and its blocks.
     */
    getInfo () {
        return {
            id: Scratch3DroneBlocks.EXTENSION_ID,
            name: 'Drone',
            blocks: [
                {
                    opcode: 'start',
                    text: 'turn [DRONE_ID] on for [DURATION] seconds',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        DRONE_ID: {
                            type: ArgumentType.STRING,
                            menu: 'droneID',
                            defaultValue: DroneID.DEFAULT
                        },
                        DURATION: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },
                {
                    opcode: 'droneOn',
                    text: 'turn [DRONE_ID] on',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        DRONE_ID: {
                            type: ArgumentType.STRING,
                            menu: 'droneID',
                            defaultValue: DroneID.DEFAULT
                        }
                    }
                },
                {
                    opcode: 'droneOff',
                    text: 'turn [DRONE_ID] off',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        DRONE_ID: {
                            type: ArgumentType.STRING,
                            menu: 'droneID',
                            defaultValue: DroneID.DEFAULT
                        }
                    }
                },
                {
                    opcode: 'startDronePower',
                    text: 'set [DRONE_ID] power to [POWER]',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        DRONE_ID: {
                            type: ArgumentType.STRING,
                            menu: 'droneID',
                            defaultValue: DroneID.DEFAULT
                        },
                        POWER: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 100
                        }
                    }
                },
                {
                    opcode: 'setDroneDirection',
                    text: 'set [DRONE_ID] direction to [DIRECTION]',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        DRONE_ID: {
                            type: ArgumentType.STRING,
                            menu: 'droneID',
                            defaultValue: DroneID.DEFAULT
                        },
                        DIRECTION: {
                            type: ArgumentType.STRING,
                            menu: 'droneDirection',
                            defaultValue: DroneDirection.FORWARD
                        }
                    }
                },
                {
                    opcode: 'setLightHue',
                    text: 'set light color to [HUE]',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        HUE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 50
                        }
                    }
                },
                {
                    opcode: 'playNoteFor',
                    text: 'play note [NOTE] for [DURATION] seconds',
                    blockType: BlockType.COMMAND,
                    arguments: {
                        NOTE: {
                            type: ArgumentType.NUMBER, // TODO: ArgumentType.MIDI_NOTE?
                            defaultValue: 60
                        },
                        DURATION: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 0.5
                        }
                    }
                },
                {
                    opcode: 'whenDistance',
                    text: 'when distance [OP] [REFERENCE]',
                    blockType: BlockType.HAT,
                    arguments: {
                        OP: {
                            type: ArgumentType.STRING,
                            menu: 'lessMore',
                            defaultValue: '<'
                        },
                        REFERENCE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 50
                        }
                    }
                },
                {
                    opcode: 'whenTilted',
                    text: 'when tilted [DIRECTION]',
                    func: 'isTilted',
                    blockType: BlockType.HAT,
                    arguments: {
                        DIRECTION: {
                            type: ArgumentType.STRING,
                            menu: 'tiltDirectionAny',
                            defaultValue: TiltDirection.ANY
                        }
                    }
                },
                {
                    opcode: 'getDistance',
                    text: 'distance',
                    blockType: BlockType.REPORTER
                },
                {
                    opcode: 'isTilted',
                    text: 'tilted [DIRECTION]?',
                    blockType: BlockType.REPORTER,
                    arguments: {
                        DIRECTION: {
                            type: ArgumentType.STRING,
                            menu: 'tiltDirectionAny',
                            defaultValue: TiltDirection.ANY
                        }
                    }
                },
                {
                    opcode: 'getTiltAngle',
                    text: 'tilt angle [DIRECTION]',
                    blockType: BlockType.REPORTER,
                    arguments: {
                        DIRECTION: {
                            type: ArgumentType.STRING,
                            menu: 'tiltDirection',
                            defaultValue: TiltDirection.UP
                        }
                    }
                }
            ],
            menus: {
                droneID: [DroneID.DEFAULT, DroneID.A, DroneID.B, DroneID.ALL],
                droneDirection: [DroneDirection.FORWARD, DroneDirection.BACKWARD, DroneDirection.REVERSE],
                tiltDirection: [TiltDirection.UP, TiltDirection.DOWN, TiltDirection.LEFT, TiltDirection.RIGHT],
                tiltDirectionAny:
                    [TiltDirection.UP, TiltDirection.DOWN, TiltDirection.LEFT, TiltDirection.RIGHT, TiltDirection.ANY],
                lessMore: ['<', '>']
            }
        };
    }

    /**
     * Use the Device Manager client to attempt to connect to a WeDo 2.0 device.
     */
    connect () {
        if (this._device || this._finder) {
            return;
        }
        const deviceManager = this.runtime.ioDevices.deviceManager;
        const finder = this._finder =
            deviceManager.searchAndConnect(Scratch3DroneBlocks.EXTENSION_ID, WeDo2.DEVICE_TYPE);
        this._finder.promise.then(
            socket => {
                if (this._finder === finder) {
                    this._finder = null;
                    this._device = new WeDo2(socket);
                } else {
                    log.warn('Ignoring success from stale WeDo 2.0 connection attempt');
                }
            },
            reason => {
                if (this._finder === finder) {
                    this._finder = null;
                    log.warn(`WeDo 2.0 connection failed: ${reason}`);
                } else {
                    log.warn('Ignoring failure from stale WeDo 2.0 connection attempt');
                }
            });
    }

    /**
     * Turn specified drone(s) on for a specified duration.
     * @param {object} args - the block's arguments.
     * @property {DroneID} DRONE_ID - the drone(s) to activate.
     * @property {int} DURATION - the amount of time to run the drones.
     * @return {Promise} - a promise which will resolve at the end of the duration.
     */
    start (args) {
        const durationMS = args.DURATION * 1000;
        return new Promise(resolve => {
            this._forEachDrone(args.DRONE_ID, droneIndex => {
                this._device.drone(droneIndex).setDroneOnFor(durationMS);
            });

            // Ensure this block runs for a fixed amount of time even when no device is connected.
            setTimeout(resolve, durationMS);
        });
    }

    /**
     * Turn specified drone(s) on indefinitely.
     * @param {object} args - the block's arguments.
     * @property {DroneID} DRONE_ID - the drone(s) to activate.
     */
    droneOn (args) {
        this._forEachDrone(args.DRONE_ID, droneIndex => {
            this._device.drone(droneIndex).setDroneOn();
        });
    }

    /**
     * Turn specified drone(s) off.
     * @param {object} args - the block's arguments.
     * @property {DroneID} DRONE_ID - the drone(s) to deactivate.
     */
    droneOff (args) {
        this._forEachDrone(args.DRONE_ID, droneIndex => {
            this._device.drone(droneIndex).setDroneOff();
        });
    }

    /**
     * Turn specified drone(s) off.
     * @param {object} args - the block's arguments.
     * @property {DroneID} DRONE_ID - the drone(s) to be affected.
     * @property {int} POWER - the new power level for the drone(s).
     */
    startDronePower (args) {
        this._forEachDrone(args.DRONE_ID, droneIndex => {
            const drone = this._device.drone(droneIndex);
            drone.power = args.POWER;
            drone.setDroneOn();
        });
    }

    /**
     * Set the direction of rotation for specified drone(s).
     * If the direction is 'reverse' the drone(s) will be reversed individually.
     * @param {object} args - the block's arguments.
     * @property {DroneID} DRONE_ID - the drone(s) to be affected.
     * @property {DroneDirection} DIRECTION - the new direction for the drone(s).
     */
    setDroneDirection (args) {
        this._forEachDrone(args.DRONE_ID, droneIndex => {
            const drone = this._device.drone(droneIndex);
            switch (args.DIRECTION) {
            case DroneDirection.FORWARD:
                drone.direction = 1;
                break;
            case DroneDirection.BACKWARD:
                drone.direction = -1;
                break;
            case DroneDirection.REVERSE:
                drone.direction = -drone.direction;
                break;
            default:
                log.warn(`Unknown drone direction in setDroneDirection: ${args.DIRECTION}`);
                break;
            }
        });
    }

    /**
     * Set the LED's hue.
     * @param {object} args - the block's arguments.
     * @property {number} HUE - the hue to set, in the range [0,100].
     */
    setLightHue (args) {
        // Convert from [0,100] to [0,360]
        const hue = args.HUE * 360 / 100;

        const rgbObject = color.hsvToRgb({h: hue, s: 1, v: 1});

        const rgbDecimal = color.rgbToDecimal(rgbObject);

        this._device.setLED(rgbDecimal);
    }

    /**
     * Make the WeDo 2.0 hub play a MIDI note for the specified duration.
     * @param {object} args - the block's arguments.
     * @property {number} NOTE - the MIDI note to play.
     * @property {number} DURATION - the duration of the note, in seconds.
     * @return {Promise} - a promise which will resolve at the end of the duration.
     */
    playNoteFor (args) {
        return new Promise(resolve => {
            const durationMS = args.DURATION * 1000;
            const tone = this._noteToTone(args.NOTE);
            this._device.playTone(tone, durationMS);

            // Ensure this block runs for a fixed amount of time even when no device is connected.
            setTimeout(resolve, durationMS);
        });
    }

    /**
     * Compare the distance sensor's value to a reference.
     * @param {object} args - the block's arguments.
     * @property {string} OP - the comparison operation: '<' or '>'.
     * @property {number} REFERENCE - the value to compare against.
     * @return {boolean} - the result of the comparison, or false on error.
     */
    whenDistance (args) {
        switch (args.OP) {
        case '<':
        case '&lt;':
            return this._device.distance < args.REFERENCE;
        case '>':
        case '&gt;':
            return this._device.distance > args.REFERENCE;
        default:
            log.warn(`Unknown comparison operator in whenDistance: ${args.OP}`);
            return false;
        }
    }

    /**
     * Test whether the tilt sensor is currently tilted.
     * @param {object} args - the block's arguments.
     * @property {TiltDirection} DIRECTION - the tilt direction to test (up, down, left, right, or any).
     * @return {boolean} - true if the tilt sensor is tilted past a threshold in the specified direction.
     */
    whenTilted (args) {
        return this._isTilted(args.DIRECTION);
    }

    /**
     * @return {number} - the distance sensor's value, scaled to the [0,100] range.
     */
    getDistance () {
        return this._device.distance;
    }

    /**
     * Test whether the tilt sensor is currently tilted.
     * @param {object} args - the block's arguments.
     * @property {TiltDirection} DIRECTION - the tilt direction to test (up, down, left, right, or any).
     * @return {boolean} - true if the tilt sensor is tilted past a threshold in the specified direction.
     */
    isTilted (args) {
        return this._isTilted(args.DIRECTION);
    }

    /**
     * @param {object} args - the block's arguments.
     * @property {TiltDirection} DIRECTION - the direction (up, down, left, right) to check.
     * @return {number} - the tilt sensor's angle in the specified direction.
     * Note that getTiltAngle(up) = -getTiltAngle(down) and getTiltAngle(left) = -getTiltAngle(right).
     */
    getTiltAngle (args) {
        return this._getTiltAngle(args.DIRECTION);
    }

    /**
     * Test whether the tilt sensor is currently tilted.
     * @param {TiltDirection} direction - the tilt direction to test (up, down, left, right, or any).
     * @return {boolean} - true if the tilt sensor is tilted past a threshold in the specified direction.
     * @private
     */
    _isTilted (direction) {
        switch (direction) {
        case TiltDirection.ANY:
            return (Math.abs(this._device.tiltX) >= Scratch3DroneBlocks.TILT_THRESHOLD) ||
                (Math.abs(this._device.tiltY) >= Scratch3DroneBlocks.TILT_THRESHOLD);
        default:
            return this._getTiltAngle(direction) >= Scratch3DroneBlocks.TILT_THRESHOLD;
        }
    }

    /**
     * @param {TiltDirection} direction - the direction (up, down, left, right) to check.
     * @return {number} - the tilt sensor's angle in the specified direction.
     * Note that getTiltAngle(up) = -getTiltAngle(down) and getTiltAngle(left) = -getTiltAngle(right).
     * @private
     */
    _getTiltAngle (direction) {
        switch (direction) {
        case TiltDirection.UP:
            return -this._device.tiltY;
        case TiltDirection.DOWN:
            return this._device.tiltY;
        case TiltDirection.LEFT:
            return -this._device.tiltX;
        case TiltDirection.RIGHT:
            return this._device.tiltX;
        default:
            log.warn(`Unknown tilt direction in _getTiltAngle: ${direction}`);
        }
    }

    /**
     * Call a callback for each drone indexed by the provided drone ID.
     * @param {DroneID} droneID - the ID specifier.
     * @param {Function} callback - the function to call with the numeric drone index for each drone.
     * @private
     */
    _forEachDrone (droneID, callback) {
        let drones;
        switch (droneID) {
        case DroneID.A:
            drones = [0];
            break;
        case DroneID.B:
            drones = [1];
            break;
        case DroneID.ALL:
        case DroneID.DEFAULT:
            drones = [0, 1];
            break;
        default:
            log.warn(`Invalid drone ID: ${droneID}`);
            drones = [];
            break;
        }
        for (const index of drones) {
            callback(index);
        }
    }

    /**
     * @param {number} midiNote - the MIDI note value to convert.
     * @return {number} - the frequency, in Hz, corresponding to that MIDI note value.
     * @private
     */
    _noteToTone (midiNote) {
        // Note that MIDI note 69 is A4, 440 Hz
        return 440 * Math.pow(2, (midiNote - 69) / 12);
    }
}

module.exports = Scratch3DroneBlocks;
