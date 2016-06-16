#
# Copyright (C) 2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=lua-nixio-memcached
PKG_VERSION:=0.13
PKG_RELEASE:=1
PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex-team.ru>
PKG_LICENSE:=BSD

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=git://github.com/ya-mouse/lua-nixio-memcached.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=HEAD
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(PKG_SOURCE_VERSION).tar.gz

PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/lua-nixio-memcached
  SUBMENU:=Lua
  SECTION:=lang
  CATEGORY:=Languages
  TITLE:=Lua memcached client for NIXIO API
  DEPENDS:=+luci-lib-nixio
endef

define Package/lua-nixio-memcached/description
  Memcached client for NIXIO API
endef

define Package/lua-nixio-memcached/install
	$(INSTALL_DIR) $(1)/usr/lib/lua/nixio
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/memcached.lua $(1)/usr/lib/lua/nixio/memcached.lua
endef

$(eval $(call BuildPackage,lua-nixio-memcached))
