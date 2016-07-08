#
# Copyright (C) 2009-2016 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=freeipmi
PKG_VERSION:=1.5.2
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.gz
PKG_SOURCE_URL:=@GNU/freeipmi
PKG_MD5SUM:=0e0adbf0ddf91f52fa5d7a5ddedb936f
PKG_MAINTAINER:=Anton D. Kachalov <mouse@yandex-team.ru>

PKG_LICENSE:=GPLv3
PKG_LICENSE_FILES:=COPYING

PKG_BUILD_PARALLEL:=1
PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/freeipmi/Default
  SECTION:=utils
  CATEGORY:=Utilities
  URL:=http://www.gnu.org/software/freeipmi/
endef

define Package/freeipmi/Default/description
  FreeIPMI provides in-band and out-of-band IPMI software
endef

define Package/freeipmi
$(call Package/freeipmi/Default)
  TITLE:=FreeIPMI (tools)
  DEPENDS:=+libfreeipmi
endef

define Package/libfreeipmi
$(call Package/freeipmi/Default)
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=FreeIPMI (libraries)
endef

define Package/libfreeipmi/description
$(call Package/freeipmi/Default/description)
 .
 This package contains shared libraries, needed by other programs.
endef

TARGET_CFLAGS += $(FPIC) -D_LIBC=0

CONFIGURE_VARS += \
	ac_cv_header_stropts_h=no \
	ac_cv_header_sys_stropts_h=no \
	

CONFIGURE_ARGS += \
	--disable-init-scripts \
	--without-encryption \
	--disable-ipmisdrcachedir \
	--without-random-device \
	

define Package/libfreeipmi/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/lib{free,}ipmi*.so.* $(1)/usr/lib/
endef

define Package/freeipmi/install
	$(INSTALL_DIR) $(1)/usr/sbin
	cp -a $(PKG_INSTALL_DIR)/usr/sbin/* $(1)/usr/sbin/
endef

$(eval $(call BuildPackage,libfreeipmi))
$(eval $(call BuildPackage,freeipmi))
