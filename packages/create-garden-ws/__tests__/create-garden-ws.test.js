'use strict';

import createGardenWs from '..';
import { strict as assert } from 'assert';

assert.strictEqual(createGardenWs(), 'Hello from createGardenWs');
console.info('createGardenWs tests passed');
