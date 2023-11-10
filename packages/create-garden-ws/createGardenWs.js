/**
 * Copyright (c) 2015-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//   /!\ DO NOT MODIFY THIS FILE /!\
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// The only job of create-garden-ws is to init the repository and then
// forward all the commands to the local version of create-garden-ws.
//
// If you need to add a new command, please add it to the scripts/ folder.
//
// The only reason to modify this file is to add more warnings and
// troubleshooting information for the `create-garden-ws` command.
//
// Do not make breaking changes! We absolutely don't want to have to
// tell people to update their global version of create-garden-ws.
//
// Also be careful with new language features.
// This file must work on Node 10+.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//   /!\ DO NOT MODIFY THIS FILE /!\
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

'use strict';

import { Command } from 'commander';
import { green, cyan, bold } from 'chalk';

import { name as _name, version } from './package.json';

let projectName;

function init() {
    const program = new Command(_name)
        .version(version)
        .arguments('<project-directory>')
        .usage(`${green('<project-directory>')} [options]`)
        .action(name => {
            projectName = name;
        })
        .option('--verbose', 'print additional logs')
        .option('--info', 'print environment debug info')
        .option(
            '--template <path-to-template>',
            'specify a template for the created workspace'
        )
        .allowUnknownOption()
        .on('--help', () => {
            console.log(
                `    Only ${green('<project-directory>')} is required.`
            );
            console.log();
            console.log(`    A custom ${cyan('--template')} can be one of:`);
            console.log(
                `      - a custom template published on npm: ${green(
                    'cgw-template'
                )}`
            );
            console.log(
                `      - a local path relative to the current working directory: ${green(
                    'file:../my-custom-template'
                )}`
            );
            console.log();
        })
        .parse(process.argv);

    if (program.info) {
        console.log(bold('\nEnvironment Info:'));
        console.log(
            `\n  current version of ${_name}: ${version}`
        );
        console.log(`  running from ${__dirname}`);
        return envinfo
            .run(
                {
                    System: ['OS', 'CPU'],
                    Binaries: ['Node', 'npm', 'Yarn'],
                    Browsers: [
                        'Chrome',
                        'Edge',
                        'Internet Explorer',
                        'Firefox',
                        'Safari',
                    ],
                    npmPackages: ['react', 'react-dom', 'react-scripts'],
                    npmGlobalPackages: ['create-react-app'],
                },
                {
                    duplicates: true,
                    showNotFound: true,
                }
            )
            .then(console.log);
    }

    if (typeof projectName === 'undefined') {
        console.error('Please specify the project directory:');
        console.log(
            `  ${cyan(program.name())} ${green('<project-directory>')}`
        );
        console.log();
        console.log('For example:');
        console.log(
            `  ${cyan(program.name())} ${green('my-garden-ws')}`
        );
        console.log();
        console.log(
            `Run ${cyan(`${program.name()} --help`)} to see all options.`
        );
        process.exit(1);
    } else {
        console.log(`####################  ${projectName}`)
    }
}
function getTemplateInstallPackage() {
    console.log('############  getTemplateInstallPackage()')
}

export default {
    init,
    getTemplateInstallPackage,
};