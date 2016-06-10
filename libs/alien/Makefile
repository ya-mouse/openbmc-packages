#
# Copyright (C) 2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=alien
PKG_VERSION:=0.7.0
PKG_RELEASE:=1
PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex-team.ru>
PKG_LICENSE:=MIT/X11

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/mascarenhas/alien
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=HEAD
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz

PKG_FIXUP:=autoreconf
PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/alien
  SUBMENU:=Lua
  SECTION:=lang
  CATEGORY:=Languages
  TITLE:=Alien lib for Lua
  DEPENDS:=+lua +libffi
endef

define Package/alien/description
  Alien lets a Lua application call load dynamic libraries and call C functions.
endef

TARGET_CFLAGS += $(FPIC)

# add make variable overrides here
MAKE_FLAGS +=

CONFIGURE_ARGS += \
	--libdir=/usr/lib/lua \
	--datadir=/usr/share/lua

define Package/alien/install
	$(INSTALL_DIR) $(1)/usr/lib/lua
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/lib/lua/alien_c.so $(1)/usr/lib/lua/
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/share/lua/alien.lua $(1)/usr/lib/lua/
endef

$(eval $(call BuildPackage,alien))
