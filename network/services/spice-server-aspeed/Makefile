#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=spice-server-aspeed
PKG_VERSION:=0.1
PKG_RELEASE:=1~git

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=git://github.com/ya-mouse/spice-server-aspeed.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=HEAD
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz

PKG_LICENSE:=GPL-3
PKG_LICENSE_FILES:=COPYING

PKG_FIXUP:=autoreconf
PKG_INSTALL:=1

PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex.ru>

include $(INCLUDE_DIR)/package.mk

CONFIGURE_ARGS += \
	--enable-shared \
	--enable-static \
	--disable-werror

CONFIGURE_VARS += \
	bash_cv_wcwidth_broken=no \
	bash_cv_func_sigsetjmp=yes

TARGET_CFLAGS += $(FPIC)

define Package/spice-server-aspeed
  SECTION:=net
  CATEGORY:=Network
  DEPENDS:=+spice
  TITLE:=SPICE server for Aspeed SoC
endef

define Package/spice-server-aspeed/description
  Spice server for Aspeed SoC
endef

define Package/spice-server-aspeed/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(CP) $(PKG_INSTALL_DIR)/usr/sbin/spice_server_aspeed $(1)/usr/sbin/spice-server-aspeed
endef

$(eval $(call BuildPackage,spice-server-aspeed))
