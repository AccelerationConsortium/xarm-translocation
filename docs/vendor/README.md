# Vendor manuals

| File | What | Source |
|---|---|---|
| `2F-85_2F-140_UR_PDF_20200211.pdf` | Robotiq 2F-85 & 2F-140 for Universal Robots, instruction manual, revision 2020-02-11 (CB-Series and e-Series) | <https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_UR_PDF_20200211.pdf> (downloaded 2026-10-08, sha256 `02d95c13…179917030`) |

The manual says the latest revision is at support.robotiq.com. Check there
before relying on a figure that may have changed.

## 2F-140 figures used on ligand_ur5e

Gripper on the UR wrist with the standard coupling, no Robotiq wrist camera or
FT sensor. Section 5.2.3, p. 127, tool flange frame:

- mass 1.025 kg
- centre of mass (0, 0, 73.0) mm
- TCP (0, 0, 244.0) mm

Inertia, fingers fully open (p. 128): diag(7400, 9320, 2260) kg·mm².
Section 3.7 (p. 37) says to set the payload and centre of gravity on the
pendant before use. Add the held part's mass when one is carried.
