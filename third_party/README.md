# Third-party decoder dependencies

The DRM and DMR modules use pinned upstream source revisions. CMake downloads
them only when an existing local source tree is not supplied or found beside
the SDR++ checkout.

| Component | Used by | Upstream | Pinned revision | License |
| --- | --- | --- | --- | --- |
| Dream | DRM decoder | `https://github.com/wwek/dream.git` | `fddf0cc69d0085d864d35e43e761de76104cfb17` | GPL-2.0-or-later |
| DSDcc | DMR decoder | `https://github.com/f4exb/dsdcc.git` | `f27b32d2df131ae3a376fe72d3fb880ae1f9ede1` | GPL-3.0-or-later |
| mbelib | DMR voice | `https://github.com/szechyjs/mbelib.git` | `9a04ed5c78176a9965f3d43f7aa1b1f5330e771f` | ISC-style permissive license |
| sdr_receiver_dvb_t2 | DVB-T2 decoder | `https://github.com/Oleg-Malyutin/sdr_receiver_dvb_t2.git` | `332e9704c29d16940a7322b9f49a11a6f5c04437` | GPL-3.0-or-later |

Our integration changes are stored as reviewable patches:

- `patches/dream-embedded-sdrpp.patch`
- `patches/dsdcc-dmr-data.patch`
- `patches/sdr_receiver_dvb_t2-ldpc.patch`

CMake applies these patches to freshly downloaded sources. Developers can use
existing dependency checkouts by setting `SDRPP_DREAM_ROOT`,
`SDRPP_DSDCC_ROOT`, and `SDRPP_MBELIB_ROOT`. For compatibility with the
original development layout, patched `../dream`, `../dsdcc`, and `../mbelib`
directories are detected automatically.

The DVB-T2 reference implementation can be overridden with
`SDRPP_DVBT2_REFERENCE_ROOT`. The existing
`external/sdr_receiver_dvb_t2` developer checkout is detected automatically.

## AMBE/mbelib notice

DSDcc's upstream documentation warns that implementations built with mbelib
may involve patents owned by Digital Voice Systems, Inc. in some jurisdictions.
The source license permits redistribution, but anyone distributing or using a
compiled DMR voice decoder is responsible for determining whether separate
patent licensing is required where they operate. This project does not grant
patent rights.

## Updating a dependency

1. Update the pinned commit in the corresponding decoder module's
   `CMakeLists.txt`.
2. Rebase and regenerate the associated patch against that exact commit.
3. Verify the patch with `git apply --check` on a clean dependency checkout.
4. Configure a clean build without sibling dependency directories to exercise
   the download-and-patch path.
5. Retest the affected decoder before committing the update.
