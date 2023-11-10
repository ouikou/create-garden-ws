'use strict';

const cgwTemplate = require('..');
const assert = require('assert').strict;

assert.strictEqual(cgwTemplate(), 'Hello from cgwTemplate');
console.info('cgwTemplate tests passed');
