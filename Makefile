################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
# Technologies AG.  SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=APPLICATION

MTB_PROJECTS=proj_cm33_s proj_cm33_ns proj_cm55

# The CM55 display/ML integration carries checked-in BSP GeneratedSource
# changes, so builds must not regenerate configurator output.
SKIP_CODE_GEN=1
export SKIP_CODE_GEN

################################################################################
# Paths
################################################################################

include common_app.mk

include $(CY_TOOLS_DIR)/make/application.mk

.PHONY: user_application_postbuild
user_application_postbuild: recipe_application_postbuild
	env -u MAKEFLAGS -u MFLAGS -u GNUMAKEFLAGS -u MAKELEVEL -u MAKEOVERRIDES -u MTB_APPLICATION_RECUSIVE $(MAKE) -B -C $(firstword $(MTB_PROJECTS)) sign_combine MTB_APPLICATION_SUBPROJECTS="$(MERGE)"
