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

module.exports = function (
    appPath,
    appName,
    verbose,
    originalDirectory,
    templateName
) {
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

    // Copy the files for the user
    const templateDir = path.join(templatePath, 'template');
    if (fs.existsSync(templateDir)) {
        fs.copySync(templateDir, appPath);
    } else {
        console.error(
            `Could not locate supplied template: ${chalk.green(templateDir)}`
        );
        return;
    }

    const vsCodeExists = fs.existsSync(path.join(appPath, '_vscode'));
    if (vsCodeExists) {
        // Rename if there's already a `_vscode` folder there
        fs.renameSync(
            path.join(appPath, '_vscode'),
            path.join(appPath, '.vscode')
        );
    }

    const gitignoreExists = fs.existsSync(path.join(appPath, '.gitignore'));
    if (gitignoreExists) {
        // Append if there's already a `.gitignore` file there
        const data = fs.readFileSync(path.join(appPath, 'gitignore'));
        fs.appendFileSync(path.join(appPath, '.gitignore'), data);
        fs.unlinkSync(path.join(appPath, 'gitignore'));
    } else {
        // Rename gitignore after the fact to prevent npm from renaming it to .npmignore
        // See: https://github.com/npm/npm/issues/1862
        fs.moveSync(
            path.join(appPath, 'gitignore'),
            path.join(appPath, '.gitignore'),
            []
        );
    }


    const genShExists = fs.existsSync(path.join(appPath, 'src', 'rulebase', '.generator', 'gen.sh'));
    if (genShExists) {
        // change the permissions of `src/rulebase/.generator/gen.sh`
        fs.chmodSync(path.join(appPath, 'src', 'rulebase', '.generator', 'gen.sh'), '777');
    }
    const mainMakeShExists = fs.existsSync(path.join(appPath, 'make.sh'));
    if (mainMakeShExists) {
        // change the permissions of `make.sh`
        fs.chmodSync(path.join(appPath, 'make.sh'), '777');
    }
    const subMakeShExists = fs.existsSync(path.join(appPath, 'src', 'rulebase', 'make.sh'));
    if (subMakeShExists) {
        // change the permissions of `src/rulebase/make.sh`
        fs.chmodSync(path.join(appPath, 'src', 'rulebase', 'make.sh'), '777');
    }

    console.log('');
    console.log('Removing template files. This might take a couple of minutes.');
    // On 'exit' we will delete these files from target directory.
    const knownGeneratedFiles = ['package.json', 'package-lock.json', 'node_modules'];
    const currentFiles = fs.readdirSync(path.join(appPath));
    currentFiles.forEach(file => {
        knownGeneratedFiles.forEach(fileToMatch => {
            // This removes all knownGeneratedFiles.
            if (file === fileToMatch) {
                console.log(`Deleting generated file.... ${chalk.cyan(file)}`);
                fs.removeSync(path.join(appPath, file));
            }
        });
    });

    console.log();
    console.log(`Success! Created ${chalk.bgGreen(appName)} at ${chalk.green(appPath)}`);
};
