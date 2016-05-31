#
# Copyright (C) 2016 Anton D. Kachalov
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=ast-helper
PKG_VERSION:=2.18.0
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/ast-helper
  SUBMENU:=Other modules
  DEPENDS:=@TARGET_aspeed
  TITLE:=Helper functions for other Aspeed drivers
  FILES:=$(PKG_BUILD_DIR)/helper.ko
  AUTOLOAD:=$(call AutoLoad,20,helper)
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		ARCH="$(LINUX_KARCH)" \
		SUBDIRS="$(PKG_BUILD_DIR)" \
		EXTRA_CFLAGS="$(BUILDFLAGS)" \
		modules
endef

$(eval $(call KernelPackage,ast-helper))
