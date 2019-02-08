## Table of Contents

* [Navigating the Repository](#navigating-the-repository)
* [Change Log](#change-log)
* [Installation](#installation)
* [Required Tools](#required-tools)
* [Examples List](#examples--demo-list)
* [References](#references)
* [FAQ](docs/faq.md)
* [Versioning Proprietary Projects](docs/suggested_workflow.md)

# Introduction

These examples and demos are for **TI SimpleLink CC13X2 SDK 2.30.00.45**

This repository contains proprietary radio sample applications for
Texas Instruments' SimpleLink CC13X2 SDK. These examples have not been
validated as production-ready.

**Do not** use GitHub's bug tracking feature for support. For inquiries, see
the forum relevant to your device and application:
[Sub-1 Ghz Forum](https://e2e.ti.com/support/wireless-connectivity/sub-1-ghz/f/156) or [Other Wireless Forum](https://e2e.ti.com/support/wireless-connectivity/other-wireless/f/667).

To use the examples and tools in this repository, please download and install
the [SimpleLink CC13X2 SDK](http://www.ti.com/tool/SIMPLELINK-CC13X2-SDK)
**first**, and if necessary [buy an evaluation kit](http://www.ti.com/tool/LAUNCHXL-CC1352R1).

If you have any questions please refer to the [FAQ page](docs/faq.md).

For examples for other SDK versions and platforms, see table below.

<table>
  <tbody>
    <tr>
      <th width = 50%>SDK</th>
      <th>Examples</th>
    </tr>
    <tr>
      <td>
        <b>TI SimpleLink CC13X2 SDK 2.30.00.45 (current)</b>
      </td>
      <td>
        <ul>
          <li><a href="#rf-sensor-and-collector">RF Sensor and Collector</a></li>
        </ul>
      </td>
    </tr>
  </tbody>
</table>

# Navigating the Repository

The examples provided on this GitHub page serve as a plugin to a corresponding
device SDK release. The master branch will always point to the latest release.

Older releases can be accessed by checking out/downloading their corresponding
branch. For more information on supported examples
please consult the readme.md of the desired branch/release.

For more information about different SDK components, please consult the
SDK documentation.

## Change Log

Note: The version numbers below are related to GitHub proprietary_examples
releases. The numbering scheme is in the form of M.mm.pp.bb. The fields pp.bb
are incremented as GitHub examples are released, M.mm will map a GitHub
release to a SimpleLink SDK release.

### 2.30.00.00
New examples added.
* RF Sensor and Collector: Synchronous time-slotted RF Sensor and Collector implemenation based on the RF Driver API.

## Installation

This repository can be cloned and tracked using Git. For instructions on how to
clone a repository from Github please refer to this guide:
[Clone from Github](https://help.github.com/articles/cloning-a-repository/)

For users who are unfamiliar with Git, there is the option of downloading the
contents of the repository as a zip file. See instructions below.

1. Click the green "Clone or download" button
1. Select "Download ZIP" option
1. Zip folder will appear in your Downloads folder

This repository can be cloned/download anywhere on your computer. There is a
dependency between this repository and the SimpleLink CC2640R2 SDK install
location.

By default the SimpleLink CC13X2 SDK will install to:

    C:\ti\simplelink_cc13x2_sdk_2_30_00_45

If the Simplelink CC13X2 SDK must be installed to a different location, then
see the [FAQ page](docs/faq.md) for IDE specific instructions for changing
environment variables.

## Required Tools

The examples in this repository currently only support the CCS toolchains.
Please refer to the release notes for the supported versions of each
toolchain. Using a non-supported version is untested and may result in
unexpected behavior.

For more information on toolchain setup, please refer to our
[FAQ page](docs/faq.md).

## FAQ

The [FAQ page](docs/faq.md) will try to address some of the common questions
related to the proprietary_examples repo.

## Examples / Demo List

### RF Sensor and Collector

A synchronous RF Sensor and Collector implementation using a proprietary
time-slotted radio protocol.

* rfSensor
    * [Documentation](examples/rtos/CC1352R1_LAUNCHXL/rfSensor/README.md)
    * [CCS Project Files](examples/rtos/CC1352R1_LAUNCHXL/rfSensor/tirtos/ccs)
    * [Source](examples/rtos/CC1352R1_LAUNCHXL/rfSensor/src)


* rfCollector
    * [Documentation](examples/rtos/CC1352R1_LAUNCHXL/rfCollector/README.md)
    * [CCS Project Files](examples/rtos/CC1352R1_LAUNCHXL/rfCollector/tirtos/ccs)
    * [Source](examples/rtos/CC1352R1_LAUNCHXL/rfCollector/src)

## References

The following reference pages may be helpful during general development using
the SimpleLink CC13X2 SDK. Users developing proprietary RF applications are
encouraged to read the [Proprietary RF User's Guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC13x2%20SDK%2FDocuments%2FProprietary%20RF%2FProprietary%20RF%20User's%20Guide).

As an additional resource, users are encouraged to complete the
[SimpleLink Academy](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC13x2%20SDK%2FSimpleLink%20Academy)
training.

