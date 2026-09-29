obj-m += zimacube_ec.o

NAME    := zimacube-ec
VERSION := $(shell sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' dkms.conf)
SRCDIR  := /usr/src/$(NAME)-$(VERSION)

# DKMS sets KERNELRELEASE when it rebuilds for a kernel other than the running
# one; without this the build would target uname -r instead.
KVER ?= $(if $(KERNELRELEASE),$(KERNELRELEASE),$(shell uname -r))
KDIR ?= /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

# --- install paths ---------------------------------------------------------

# everything except the module: autoload, curve config, apply script, unit
install-support:
	install -D -m 0644 zimacube-ec.modules-load /etc/modules-load.d/$(NAME).conf
	install -D -m 0755 apply-fan-curve.sh /usr/local/sbin/zimacube-apply-fan-curve
	install -D -m 0644 zimacube-fan-curve.service /etc/systemd/system/zimacube-fan-curve.service
	@test -e /etc/zimacube-fan-curve.conf || install -D -m 0644 fan-curve.conf /etc/zimacube-fan-curve.conf

# plain module install: breaks on the next kernel upgrade, prefer `make dkms`
install: all install-support
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a
	@echo
	@echo "Now: systemctl daemon-reload && systemctl enable --now zimacube-fan-curve"

# --- dkms -----------------------------------------------------------------

check-version:
	@test -n "$(VERSION)" || { echo "could not read PACKAGE_VERSION from dkms.conf" >&2; exit 1; }

# install or update; safe to re-run after editing the source
dkms: check-version install-support
	install -D -m 0644 zimacube_ec.c $(SRCDIR)/zimacube_ec.c
	install -D -m 0644 Makefile          $(SRCDIR)/Makefile
	install -D -m 0644 dkms.conf         $(SRCDIR)/dkms.conf
	@dkms status -m $(NAME) -v $(VERSION) | grep -q . || dkms add -m $(NAME) -v $(VERSION)
	# build --force is required: install --force alone reinstalls the cached
	# object and silently skips compiling an edited source
	dkms build   -m $(NAME) -v $(VERSION) --force
	dkms install -m $(NAME) -v $(VERSION) --force
	@echo
	@echo "Then: make reload   (and enable zimacube-fan-curve if you want the curve applied at boot)"

dkms-remove: check-version
	@dkms remove -m $(NAME) -v $(VERSION) --all 2>/dev/null || true
	rm -rf $(SRCDIR)

# drop every version, including ones left over from earlier installs
dkms-purge:
	@for v in $$(dkms status -m $(NAME) | sed -n 's/^$(NAME)[/,] *\([^,:]*\).*/\1/p' | sort -u); do \
		echo "removing $(NAME)/$$v"; \
		dkms remove -m $(NAME) -v $$v --all 2>/dev/null || true; \
	done
	rm -rf /usr/src/$(NAME)-*

reload:
	@if lsmod | grep -q '^zimacube_ec_fan '; then echo 'unload legacy zimacube_ec_fan before loading zimacube_ec' >&2; exit 1; fi
	@if lsmod | grep -q '^zimacube_ec '; then rmmod zimacube_ec; fi
	modprobe zimacube_ec
	@dmesg | grep -i zimacube | tail -4

uninstall:
	rm -f /etc/modules-load.d/$(NAME).conf
	rm -f /usr/local/sbin/zimacube-apply-fan-curve
	rm -f /etc/systemd/system/zimacube-fan-curve.service
	@echo "left /etc/zimacube-fan-curve.conf and any installed module in place"
	@echo "for the module: make dkms-remove   (or dkms-purge)"

.PHONY: all clean install install-support check-version dkms dkms-remove dkms-purge reload uninstall
