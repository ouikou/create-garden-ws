// @remove-file-on-eject
/**
 * Copyright (c) 2015-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
'use strict';

// Makes the script crash on unhandled rejections instead of silently
// ignoring them. In the future, promise rejections that are not handled will
// terminate the Node.js process with a non-zero exit code.
process.on('unhandledRejection', err => {
    throw err;
});

const fs = require('fs-extra');
const path = require('path');
const chalk = require('chalk');
const execSync = require('child_process').execSync;
const os = require('os');

function isInGitRepository() {
    try {
        execSync('git rev-parse --is-inside-work-tree', { stdio: 'ignore' });
        return true;
    } catch (e) {
        return false;
    }
}

function isInMercurialRepository() {
    try {
        execSync('hg --cwd . root', { stdio: 'ignore' });
        return true;
    } catch (e) {
        return false;
    }
}

function tryGitInit() {
    try {
        execSync('git --version', { stdio: 'ignore' });
        if (isInGitRepository() || isInMercurialRepository()) {
            return false;
        }

        execSync('git init', { stdio: 'ignore' });
        return true;
    } catch (e) {
        console.warn('Git repo not initialized', e);
        return false;
    }
}

function tryGitCommit(appPath) {
    try {
        execSync('git add -A', { stdio: 'ignore' });
        execSync('git commit -m "Initialize project using Create GARDEN Workspace"', {
            stdio: 'ignore',
        });
        return true;
    } catch (e) {
        // We couldn't commit in already initialized git repo,
        // maybe the commit author config is not set.
        // In the future, we might supply our own committer
        // like Ember CLI does, but for now, let's just
        // remove the Git files to avoid a half-done state.
        console.warn('Git commit not created', e);
        console.warn('Removing .git directory...');
        try {
            // unlinkSync() doesn't work on directories.
            fs.removeSync(path.join(appPath, '.git'));
        } catch (removeErr) {
            // Ignore.
        }
        return false;
    }
}

module.exports = function (
    appPath,
    appName,
    verbose,
    originalDirectory,
    templateName
) {
    const appPackage = require(path.join(appPath, 'package.json'));

    if (!templateName) {
        console.log('');
        console.error(
            `A template was not provided. This is likely because you're using an outdated version of ${chalk.cyan(
                'create-garden-ws'
            )}.`
        );
        console.error(
            `Please note that global installs of ${chalk.cyan(
                'create-garden-ws'
            )} are no longer supported.`
        );
        console.error(
            `You can fix this by running ${chalk.cyan(
                'npm uninstall -g create-garden-ws'
            )} or ${chalk.cyan(
                'yarn global remove create-garden-ws'
            )} before using ${chalk.cyan('create-garden-ws')} again.`
        );
        return;
    }

    const templatePath = path.dirname(
        require.resolve(`${templateName}/package.json`, { paths: [appPath] })
    );

    const templateJsonPath = path.join(templatePath, 'template.json');

    let templateJson = {};
    if (fs.existsSync(templateJsonPath)) {
        templateJson = require(templateJsonPath);
    }

    const templatePackage = templateJson.package || {};

    console.log(`#################################   ${templateJsonPath}`)

    // This was deprecated in CRA v5.
    if (templateJson.dependencies || templateJson.scripts) {
        console.log();
        console.log(
            chalk.red(
                'Root-level `dependencies` and `scripts` keys in `template.json` were deprecated for Create GARDEN Workspace 1.\n' +
                'This template needs to be updated to use the new `package` key.'
            )
        );
        console.log('For more information, visit https://cra.link/templates');
    }
};