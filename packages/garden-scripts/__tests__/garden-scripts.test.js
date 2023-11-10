'use strict';

const gardenScripts = require('..');
const assert = require('assert').strict;

assert.strictEqual(gardenScripts(), 'Hello from gardenScripts');
console.info('gardenScripts tests passed');
