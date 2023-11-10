'use strict';

const createGardenWs = require('..');
const assert = require('assert').strict;

assert.strictEqual(createGardenWs(), 'Hello from createGardenWs');
console.info('createGardenWs tests passed');
