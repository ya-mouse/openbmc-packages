#
# Copyright (C) 2015 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=spice-protocol
PKG_VERSION:=0.16.0
PKG_RELEASE:=1~git

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=git://github.com/ya-mouse/spice-protocol.git
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
	--disable-celt051 \
	--without-sasl \
	--disable-werror

CONFIGURE_VARS += \
	bash_cv_wcwidth_broken=no \
	bash_cv_func_sigsetjmp=yes

TARGET_CFLAGS += $(FPIC)

define Package/spice-protocol
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=SPICE core protocol definition
  URL:=http://spice-space.org/
endef

define Package/spice-protocol/description
 The Simple Protocol for Independent Computing Environments (SPICE) is
 a remote display system built for virtual environments which allows
 you to view a computing 'desktop' environment not only on the machine
 where it is running, but from anywhere on the Internet and from a wide
 variety of machine architectures.
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr/include/spice-1/spice
	$(CP) $(PKG_INSTALL_DIR)/usr/include/spice-1/spice/*.h $(1)/usr/include/spice-1/spice/
	$(INSTALL_DIR) $(1)/usr/share/pkgconfig
	$(CP) $(PKG_INSTALL_DIR)/usr/share/pkgconfig/spice-protocol.pc $(1)/usr/share/pkgconfig/
	$(INSTALL_DIR) $(1)/usr/share/spice-protocol
	$(CP) $(PKG_INSTALL_DIR)/usr/share/spice-protocol/* $(1)/usr/share/spice-protocol/
endef

$(eval $(call BuildPackage,spice-protocol))
