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

const https = require('https');
const chalk = require('chalk');
const commander = require('commander');
const envinfo = require('envinfo');
const execSync = require('child_process').execSync;
const fs = require('fs-extra');
const path = require('path');
const semver = require('semver');
const spawn = require('cross-spawn');
const tmp = require('tmp');
const unpack = require('tar-pack').unpack;
const validateProjectName = require('validate-npm-package-name');

const packageJson = require('./package.json');

let projectName;

function init() {
    const program = new commander.Command(packageJson.name)
        .version(packageJson.version)
        .arguments('<project-directory>')
        .usage(`${chalk.green('<project-directory>')} [options]`)
        .action(name => {
            projectName = name;
        })
        .option('--verbose', 'print additional logs')
        .option('--info', 'print environment debug info')
        .option(
            '--template <path-to-template>',
            'specify a template for the created workspace'
        )
        .option('--init-ws', 'set up a existing workspace.')
        .allowUnknownOption()
        .on('--help', () => {
            console.log(
                `    Only ${chalk.green('<project-directory>')} is required.`
            );
            console.log();
            console.log(`    A custom ${chalk.cyan('--template')} can be one of:`);
            console.log(
                `      - a custom template published on npm: ${chalk.green(
                    'cgw-template'
                )}`
            );
            console.log(
                `      - a local path relative to the current working directory: ${chalk.green(
                    'file:../my-custom-template'
                )}`
            );
            console.log();
        })
        .parse(process.argv);

    const options = program.opts();

    if (options.info) {
        console.log(chalk.bold('\nEnvironment Info:'));
        console.log(
            `\n  current version of ${packageJson.name}: ${packageJson.version}`
        );
        console.log(`  running from ${__dirname}`);
        return envinfo
            .run(
                {
                    System: ['OS', 'CPU'],
                    Binaries: ['Node', 'npm'],
                    Browsers: [
                        'Chrome',
                        'Edge',
                        'Internet Explorer',
                        'Firefox',
                        'Safari',
                    ],
                    npmGlobalPackages: ['create-garden-ws'],
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
            `  ${chalk.cyan(options.name())} ${chalk.green('<project-directory>')}`
        );
        console.log();
        console.log('For example:');
        console.log(
            `  ${chalk.cyan(options.name())} ${chalk.green('my-garden-ws')}`
        );
        console.log();
        console.log(
            `Run ${chalk.cyan(`${options.name()} --help`)} to see all options.`
        );
        process.exit(1);
    }

    // We first check the registry directly via the API, and if that fails, we try
    // the slower `npm view [package] version` command.
    //
    // This is important for users in environments where direct access to npm is
    // blocked by a firewall, and packages are provided exclusively via a private
    // registry.
    checkForLatestVersion()
        .catch(() => {
            try {
                return execSync('npm view create-garden-ws version').toString().trim();
            } catch (e) {
                return null;
            }
        })
        .then(latest => {
            if (latest && semver.lt(packageJson.version, latest)) {
                console.log();
                console.error(
                    chalk.yellow(
                        `You are running \`create-garden-ws\` ${packageJson.version}, which is behind the latest release (${latest}).\n\n` +
                        'We recommend always using the latest version of create-garden-ws if possible.'
                    )
                );
                console.log();
            } else {
                createApp(
                    projectName,
                    options.initWs,
                    options.verbose,
                    options.scriptsVersion,
                    options.template
                );
            }
        });
}

function createApp(name, initWs, verbose, version, template) {
    const unsupportedNodeVersion = !semver.satisfies(
        // Coerce strings with metadata (i.e. `15.0.0-nightly`).
        semver.coerce(process.version),
        '>=18'
    );

    if (unsupportedNodeVersion) {
        console.log(
            chalk.yellow(
                `You are using Node ${process.version} so the project will be bootstrapped with an old unsupported version of tools.\n\n` +
                `Please update to Node 14 or higher for a better, fully supported experience.\n`
            )
        );
    }

    const root = path.resolve(name);
    const appName = path.basename(root);

    checkAppName(appName);
    if (!initWs) {
        fs.ensureDirSync(name);
    }
    if (!isSafeToCreateProjectIn(root, name)) {
        process.exit(1);
    }
    console.log();

    console.log(`Creating a new GARDEN workspace in ${chalk.green(root)}.`);
    console.log();

    const originalDirectory = process.cwd();
    process.chdir(root);
    if (!checkThatNpmCanReadCwd()) {
        process.exit(1);
    }

    run(
        root,
        appName,
        version,
        verbose,
        originalDirectory,
        template
    );
}

function install(dependencies, verbose) {
    return new Promise((resolve, reject) => {
        let command = 'npm';
        let args = [
            'install',
            '--no-audit',
            '--save',
            '--save-exact',
            '--loglevel',
            'error',
        ].concat(dependencies);

        if (verbose) {
            args.push('--verbose');
        }

        const child = spawn(command, args, { stdio: 'inherit' });
        child.on('close', code => {
            if (code !== 0) {
                reject({
                    command: `${command} ${args.join(' ')}`,
                });
                return;
            }
            resolve();
        });
    });
}

function run(
    root,
    appName,
    version,
    verbose,
    originalDirectory,
    template
) {
    Promise.all([
        getInstallPackage(version, originalDirectory),
        getTemplateInstallPackage(template, originalDirectory),
    ]).then(([packageToInstall, templateToInstall]) => {
        const allDependencies = [packageToInstall];

        console.log('Installing packages. This might take a couple of minutes.');

        Promise.all([
            getPackageInfo(packageToInstall),
            getPackageInfo(templateToInstall),
        ])
            .then(([packageInfo, templateInfo]) => {
                let packageVersion = semver.coerce(packageInfo.version);

                const templatesVersionMinimum = '1.0.0';

                // Assume compatibility if we can't test the version.
                if (!semver.valid(packageVersion)) {
                    packageVersion = templatesVersionMinimum;
                }

                // Only support templates when used alongside new garden-scripts versions.
                const supportsTemplates = semver.gte(
                    packageVersion,
                    templatesVersionMinimum
                );
                if (supportsTemplates) {
                    allDependencies.push(templateToInstall);
                } else if (template) {
                    console.log('');
                    console.log(
                        `The ${chalk.cyan(packageInfo.name)} version you're using ${packageInfo.name === 'garden-scripts' ? 'is not' : 'may not be'
                        } compatible with the ${chalk.cyan('--template')} option.`
                    );
                    console.log('');
                }

                console.log(
                    `Installing ${chalk.cyan(packageInfo.name)}${supportsTemplates ? ` with ${chalk.cyan(templateInfo.name)}` : ''
                    }...`
                );
                console.log();

                return install(
                    allDependencies,
                    verbose,
                    true
                ).then(() => ({
                    packageInfo,
                    supportsTemplates,
                    templateInfo,
                }));
            })
            .then(async ({ packageInfo, supportsTemplates, templateInfo }) => {
                const packageName = packageInfo.name;
                const templateName = supportsTemplates ? templateInfo.name : undefined;
                checkNodeVersion(packageName);

                await executeNodeScript(
                    {
                        cwd: process.cwd(),
                        args: []
                    },
                    [root, appName, verbose, originalDirectory, templateName],
                    `
                  const init = require('${packageName}/scripts/init.js');
                  init.apply(null, JSON.parse(process.argv[1]));
                `
                );
            })
            .catch(reason => {
                console.log();
                console.log('Aborting installation.');
                if (reason.command) {
                    console.log(`  ${chalk.cyan(reason.command)} has failed.`);
                } else {
                    console.log(
                        chalk.red('Unexpected error. Please report it as a bug:')
                    );
                    console.log(reason);
                }
                console.log();

                // On 'exit' we will delete these files from target directory.
                const knownGeneratedFiles = ['package.json', 'node_modules'];
                const currentFiles = fs.readdirSync(path.join(root));
                currentFiles.forEach(file => {
                    knownGeneratedFiles.forEach(fileToMatch => {
                        // This removes all knownGeneratedFiles.
                        if (file === fileToMatch) {
                            console.log(`Deleting generated file... ${chalk.cyan(file)}`);
                            fs.removeSync(path.join(root, file));
                        }
                    });
                });
                const remainingFiles = fs.readdirSync(path.join(root));
                if (!remainingFiles.length) {
                    // Delete target folder if empty
                    console.log(
                        `Deleting ${chalk.cyan(`${appName}/`)} from ${chalk.cyan(
                            path.resolve(root, '..')
                        )}`
                    );
                    process.chdir(path.resolve(root, '..'));
                    fs.removeSync(path.join(root));
                }
                console.log('Done.');
                process.exit(1);
            })
    });
}

function getInstallPackage(version, originalDirectory) {
    let packageToInstall = 'garden-scripts';
    const validSemver = semver.valid(version);
    if (validSemver) {
        packageToInstall += `@${validSemver}`;
    } else if (version) {
        if (version[0] === '@' && !version.includes('/')) {
            packageToInstall += version;
        } else if (version.match(/^file:/)) {
            packageToInstall = `file:${path.resolve(
                originalDirectory,
                version.match(/^file:(.*)?$/)[1]
            )}`;
        } else {
            // for tar.gz or alternative paths
            packageToInstall = version;
        }
    }

    return Promise.resolve(packageToInstall);
}

function getTemplateInstallPackage(template, originalDirectory) {
    let templateToInstall = 'cgw-template';
    if (template) {
        if (template.match(/^file:/)) {
            templateToInstall = `file:${path.resolve(
                originalDirectory,
                template.match(/^file:(.*)?$/)[1]
            )}`;
        } else if (
            template.includes('://') ||
            template.match(/^.+\.(tgz|tar\.gz)$/)
        ) {
            // for tar.gz or alternative paths
            templateToInstall = template;
        } else {
            // Add prefix 'cgw-template-' to non-prefixed templates, leaving any
            // @scope/ and @version intact.
            const packageMatch = template.match(/^(@[^/]+\/)?([^@]+)?(@.+)?$/);
            const scope = packageMatch[1] || '';
            const templateName = packageMatch[2] || '';
            const version = packageMatch[3] || '';

            if (
                templateName === templateToInstall ||
                templateName.startsWith(`${templateToInstall}-`)
            ) {
                // Covers:
                // - cgw-template
                // - @SCOPE/cgw-template
                // - cgw-template-NAME
                // - @SCOPE/cgw-template-NAME
                templateToInstall = `${scope}${templateName}${version}`;
            } else if (version && !scope && !templateName) {
                // Covers using @SCOPE only
                templateToInstall = `${version}/${templateToInstall}`;
            } else {
                // Covers templates without the `cgw-template` prefix:
                // - NAME
                // - @SCOPE/NAME
                templateToInstall = `${scope}${templateToInstall}-${templateName}${version}`;
            }
        }
    }

    return Promise.resolve(templateToInstall);
}

function getTemporaryDirectory() {
    return new Promise((resolve, reject) => {
        // Unsafe cleanup lets us recursively delete the directory if it contains
        // contents; by default it only allows removal if it's empty
        tmp.dir({ unsafeCleanup: true }, (err, tmpdir, callback) => {
            if (err) {
                reject(err);
            } else {
                resolve({
                    tmpdir: tmpdir,
                    cleanup: () => {
                        try {
                            callback();
                        } catch (ignored) {
                            // Callback might throw and fail, since it's a temp directory the
                            // OS will clean it up eventually...
                        }
                    },
                });
            }
        });
    });
}

function extractStream(stream, dest) {
    return new Promise((resolve, reject) => {
        stream.pipe(
            unpack(dest, err => {
                if (err) {
                    reject(err);
                } else {
                    resolve(dest);
                }
            })
        );
    });
}

// Extract package name from tarball url or path.
async function getPackageInfo(installPackage) {
    if (installPackage.match(/^.+\.(tgz|tar\.gz)$/)) {
        try {
            const obj = await getTemporaryDirectory();
            let stream;
            if (/^http/.test(installPackage)) {
                stream = hyperquest(installPackage);
            } else {
                stream = fs.createReadStream(installPackage);
            }
            await extractStream(stream, obj.tmpdir);
            const obj_1 = obj;
            const { name, version } = require(path.join(
                obj_1.tmpdir,
                'package.json'
            ));
            obj_1.cleanup();
            return { name, version };
        } catch (err) {
            console.log(
                `Could not extract the package name from the archive: ${err.message}`
            );
            const assumedProjectName = installPackage.match(
                /^.+\/(.+?)(?:-\d+.+)?\.(tgz|tar\.gz)$/
            )[1];
            console.log(
                `Based on the filename, assuming it is "${chalk.cyan(
                    assumedProjectName
                )}"`
            );
            return await Promise.resolve({ name: assumedProjectName });
        }
    } else if (installPackage.startsWith('git+')) {
        // Pull package name out of git urls e.g:
        // git+https://github.com/mycompany/garden-scripts.git
        // git+ssh://github.com/mycompany/garden-scripts.git#v1.2.3
        return Promise.resolve({
            name: installPackage.match(/([^/]+)\.git(#.*)?$/)[1],
        });
    } else if (installPackage.match(/.+@/)) {
        // Do not match @scope/ when stripping off @version or @tag
        return Promise.resolve({
            name: installPackage.charAt(0) + installPackage.substr(1).split('@')[0],
            version: installPackage.split('@')[1],
        });
    } else if (installPackage.match(/^file:/)) {
        const installPackagePath = installPackage.match(/^file:(.*)?$/)[1];
        const { name, version } = require(path.join(
            installPackagePath,
            'package.json'
        ));
        return Promise.resolve({ name, version });
    }
    return Promise.resolve({ name: installPackage });
}

function checkForLatestVersion() {
    return new Promise((resolve, reject) => {
        https
            .get(
                'https://registry.npmjs.org/-/package/create-garden-ws/dist-tags',
                res => {
                    if (res.statusCode === 200) {
                        let body = '';
                        res.on('data', data => (body += data));
                        res.on('end', () => {
                            resolve(JSON.parse(body).latest);
                        });
                    } else {
                        reject();
                    }
                }
            )
            .on('error', () => {
                reject();
            });
    });
}

function checkNodeVersion(packageName) {
    const packageJsonPath = path.resolve(
        process.cwd(),
        'node_modules',
        packageName,
        'package.json'
    );

    if (!fs.existsSync(packageJsonPath)) {
        return;
    }

    const packageJson = require(packageJsonPath);
    if (!packageJson.engines || !packageJson.engines.node) {
        return;
    }

    if (!semver.satisfies(process.version, packageJson.engines.node)) {
        console.error(
            chalk.red(
                'You are running Node %s.\n' +
                'Create GARDEN Workspace requires Node %s or higher. \n' +
                'Please update your version of Node.'
            ),
            process.version,
            packageJson.engines.node
        );
        process.exit(1);
    }
}

function checkAppName(appName) {
    const validationResult = validateProjectName(appName);
    if (!validationResult.validForNewPackages) {
        console.error(
            chalk.red(
                `Cannot create a project named ${chalk.green(
                    `"${appName}"`
                )} because of npm naming restrictions:\n`
            )
        );
        [
            ...(validationResult.errors || []),
            ...(validationResult.warnings || []),
        ].forEach(error => {
            console.error(chalk.red(`  * ${error}`));
        });
        console.error(chalk.red('\nPlease choose a different project name.'));
        process.exit(1);
    }

    // TODO: there should be a single place that holds the dependencies
    const dependencies = ['create-garden-ws', 'cgw-template', 'garden-scripts'].sort();
    if (dependencies.includes(appName)) {
        console.error(
            chalk.red(
                `Cannot create a project named ${chalk.green(
                    `"${appName}"`
                )} because a dependency with the same name exists.\n` +
                `Due to the way npm works, the following names are not allowed:\n\n`
            ) +
            chalk.cyan(dependencies.map(depName => `  ${depName}`).join('\n')) +
            chalk.red('\n\nPlease choose a different project name.')
        );
        process.exit(1);
    }
}

// If project only contains files generated by GH, it’s safe.
// Also, if project contains remnant error logs from a previous
// installation, lets remove them now.
function isSafeToCreateProjectIn(root, name) {
    const validFiles = [
        '.DS_Store',
        '.git',
        '.gitattributes',
        '.gitignore',
        '.gitlab-ci.yml',
        '.hg',
        '.hgcheck',
        '.hgignore',
        '.idea',
        '.npmignore',
        '.travis.yml',
        'docs',
        'LICENSE',
        'README.md',
        'mkdocs.yml',
        'Thumbs.db',
    ];
    // These files should be allowed to remain on a failed install, but then
    // silently removed during the next create.
    const errorLogFilePatterns = [
        'npm-debug.log',
        'yarn-error.log',
        'yarn-debug.log',
    ];
    const isErrorLog = file => {
        return errorLogFilePatterns.some(pattern => file.startsWith(pattern));
    };

    const conflicts = fs
        .readdirSync(root)
        .filter(file => !validFiles.includes(file))
        // IntelliJ IDEA creates module files before CGW is launched
        .filter(file => !/\.iml$/.test(file))
        // Don't treat log files from previous installation as conflicts
        .filter(file => !isErrorLog(file));

    if (conflicts.length > 0) {
        console.log(
            `The directory ${chalk.green(name)} contains files that could conflict:`
        );
        console.log();
        for (const file of conflicts) {
            try {
                const stats = fs.lstatSync(path.join(root, file));
                if (stats.isDirectory()) {
                    console.log(`  ${chalk.blue(`${file}/`)}`);
                } else {
                    console.log(`  ${file}`);
                }
            } catch (e) {
                console.log(`  ${file}`);
            }
        }
        console.log();
        console.log(
            'Either try using a new directory name, or remove the files listed above.'
        );

        return false;
    }

    // Remove any log files from a previous installation.
    fs.readdirSync(root).forEach(file => {
        if (isErrorLog(file)) {
            fs.removeSync(path.join(root, file));
        }
    });
    return true;
}

function checkThatNpmCanReadCwd() {
    const cwd = process.cwd();
    let childOutput = null;
    try {
        // Note: intentionally using spawn over exec since
        // the problem doesn't reproduce otherwise.
        // `npm config list` is the only reliable way I could find
        // to reproduce the wrong path. Just printing process.cwd()
        // in a Node process was not enough.
        childOutput = spawn.sync('npm', ['config', 'list']).output.join('');
    } catch (err) {
        // Something went wrong spawning node.
        // Not great, but it means we can't do this check.
        // We might fail later on, but let's continue.
        return true;
    }
    if (typeof childOutput !== 'string') {
        return true;
    }
    const lines = childOutput.split('\n');
    // `npm config list` output includes the following line:
    // "; cwd = C:\path\to\current\dir" (unquoted)
    // I couldn't find an easier way to get it.
    const prefix = '; cwd = ';
    const line = lines.find(line => line.startsWith(prefix));
    if (typeof line !== 'string') {
        // Fail gracefully. They could remove it.
        return true;
    }
    const npmCWD = line.substring(prefix.length);
    if (npmCWD === cwd) {
        return true;
    }
    console.error(
        chalk.red(
            `Could not start an npm process in the right directory.\n\n` +
            `The current directory is: ${chalk.bold(cwd)}\n` +
            `However, a newly started npm process runs in: ${chalk.bold(
                npmCWD
            )}\n\n` +
            `This is probably caused by a misconfigured system terminal shell.`
        )
    );
    if (process.platform === 'win32') {
        console.error(
            chalk.red(`On Windows, this can usually be fixed by running:\n\n`) +
            `  ${chalk.cyan(
                'reg'
            )} delete "HKCU\\Software\\Microsoft\\Command Processor" /v AutoRun /f\n` +
            `  ${chalk.cyan(
                'reg'
            )} delete "HKLM\\Software\\Microsoft\\Command Processor" /v AutoRun /f\n\n` +
            chalk.red(`Try to run the above two lines in the terminal.\n`) +
            chalk.red(
                `To learn more about this problem, read: https://blogs.msdn.microsoft.com/oldnewthing/20071121-00/?p=24433/`
            )
        );
    }
    return false;
}

function executeNodeScript({ cwd, args }, data, source) {
    return new Promise((resolve, reject) => {
        const child = spawn(
            process.execPath,
            [...args, '-e', source, '--', JSON.stringify(data)],
            { cwd, stdio: 'inherit' }
        );

        child.on('close', code => {
            if (code !== 0) {
                reject({
                    command: `node ${args.join(' ')}`,
                });
                return;
            }
            resolve();
        });
    });
}

module.exports = {
    init,
    getTemplateInstallPackage,
};