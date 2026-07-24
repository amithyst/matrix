# Matrix Item Asset Pack v1

`matrix-item-asset-pack/v1` 是通用物品资产的不可变边界。枪械、家具、工具或
其他 benchmark 资产都走同一套清单、校验和 DTO；运行时不根据物品类别写分支。
本层只验证和解析已有文件，不下载、不转码，也不猜测未知格式。

## 两层清单

- asset pack 描述资产来源、license、坐标系、文件、物理和视觉部件。
- inventory 描述本次运行选择的 pack/item、运行时 `slot_id`、pool 和 spawn。

asset pack schema 是 `config/schemas/matrix-item-asset-pack-v1.schema.json`，
inventory schema 是 `config/schemas/matrix-item-inventory-v1.schema.json`。
两个 schema 和 Python 解析器都拒绝未知字段。Python 解析器额外拒绝 duplicate
key、`NaN`/`Infinity`、非 UTF-8、路径逃逸、符号链接、非普通文件以及 size/SHA256
不符。

## 内容寻址 registry

pack digest 是严格校验并归一化后的清单 canonical JSON 的 SHA256：

```text
sha256(JSON(sort_keys=true, compact=true, allow_nan=false))
```

清单只含相对路径和文件的 size/SHA256，不含本机绝对路径，所以同一个 pack
复制到不同机器后 digest 不变。registry 固定布局：

```text
<registry>/sha256/<digest前两位>/<digest>/matrix-item-asset-pack.json
```

解析器会重新计算清单 digest，并逐个打开和校验文件；目录或文件中的任何
symlink 都会 fail closed。

## 最小示例

```json
{
  "schema": "matrix-item-asset-pack/v1",
  "pack": {
    "pack_id": "benchmark.example-props",
    "revision": "2026.07.23",
    "license": {
      "spdx_id": "CC0-1.0",
      "attribution": ""
    },
    "provenance": {
      "source_name": "Example Props Benchmark",
      "source_uri": "https://example.invalid/props",
      "source_revision": "dataset-v1",
      "source_item_ids": ["crate-001"]
    },
    "coordinate_frame": {
      "up_axis": "+Z",
      "forward_axis": "+X",
      "handedness": "right",
      "meters_per_unit": 1.0
    },
    "files": [
      {
        "file_id": "crate_mesh",
        "path": "meshes/crate.stl",
        "size_bytes": 1234,
        "sha256": "<64位小写SHA256>",
        "role": "visual_mesh",
        "media_type": "model/stl",
        "format": "stl"
      }
    ],
    "items": [
      {
        "item_id": "crate",
        "label": "Crate",
        "physics": {
          "mass_kg": 4.0,
          "collision": {
            "shape": "box",
            "half_extents_m": [0.25, 0.25, 0.25]
          }
        },
        "visual_parts": [
          {
            "part_id": "body",
            "file_id": "crate_mesh",
            "rgba": [0.7, 0.5, 0.3, 1.0],
            "scale": [1.0, 1.0, 1.0],
            "translation_m": [0.0, 0.0, 0.0],
            "rotation_wxyz": [1.0, 0.0, 0.0, 0.0]
          }
        ]
      }
    ]
  }
}
```

license 当前接受一个 SPDX identifier（如 `CC0-1.0`、`MIT`）或项目自定义的
`LicenseRef-*`。复合 SPDX expression 暂未进入 v1，不能静默当作普通字符串。
provenance 必须包含原 benchmark/repository URI、revision 和原始 item id。
`files[]` 同时覆盖 mesh、license、provenance、material、texture 和 auxiliary；
每项都必须声明固定 `role`、`media_type`、size 与 SHA256。因此 `License.txt`、
来源说明和纹理可以与几何一起进入 pack digest。只有 `role=visual_mesh` 的文件
能够被 `visual_parts` 引用。

inventory 示例：

```json
{
  "schema": "matrix-item-inventory/v1",
  "inventory": {
    "inventory_id": "trna-test",
    "entries": [
      {
        "slot_id": "crate",
        "pack_digest": "sha256:<64位小写SHA256>",
        "item_id": "crate",
        "pool_size": 8,
        "spawn": {
          "distance_m": 1.0,
          "height_m": 1.0,
          "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]
        }
      }
    ]
  }
}
```

## API 与 adapter 边界

```python
from matrix_item_asset_pack import resolve_inventory

resolved = resolve_inventory(inventory_json, registry_root)
for slot in resolved.items:
    print(slot.slot_id, slot.pack.digest_ref, slot.item.visual_parts)
```

`resolve_inventory()` 返回冻结 DTO，且其中每个 `VerifiedAssetFile.path` 已完成
regular-file、size 和 SHA256 校验。`legacy_injector_specs()` 可把兼容项映射成当前
MuJoCo injector 所需字段；它只接受 canonical Matrix 坐标系、STL、无局部变换。
GLB/OBJ/FBX/USD 等资产仍可被通用 renderer adapter 消费，但旧 injector 会明确
报错，避免丢掉坐标或 transform 语义。格式转换应由独立、可审计的离线 importer
完成，并生成新的 pack 与 digest。

## Recipe-driven importer

`scripts/matrix_item_asset_import.py` 把 benchmark 或资产仓库里的原始文件复制为
不可变 pack。它不下载、不转码，也不从扩展名猜 metadata。Recipe schema 是
`config/schemas/matrix-item-asset-import-recipe-v1.schema.json`，顶层固定为
`schema` 和 `import`：

- `import.pack` 提供最终 pack 的 `pack_id`、revision、license、provenance、
  coordinate frame 和 items；其中不写 files 的 size/SHA。
- `import.files[]` 显式声明 `file_id`、相对 `source_path`、pack 内
  `target_path`、`role`、`media_type` 和 `format`。
- `source_path` 只相对于命令行的 `--source-root` 解析。因此 recipe 可以放在
  仓库 `config/items/import-recipes/`，TRNA 的原始资产仍放在独立 artifact
  目录，不写机器相关绝对路径。

例如，一个 STL mesh、license 和 provenance evidence 可以这样映射：

```json
{
  "schema": "matrix-item-asset-import-recipe/v1",
  "import": {
    "pack": {
      "pack_id": "benchmark.example-props",
      "revision": "dataset-v1",
      "license": {"spdx_id": "CC0-1.0", "attribution": ""},
      "provenance": {
        "source_name": "Example Props Benchmark",
        "source_uri": "https://example.invalid/props",
        "source_revision": "release-1",
        "source_item_ids": ["crate-001"]
      },
      "coordinate_frame": {
        "up_axis": "+Z",
        "forward_axis": "+X",
        "handedness": "right",
        "meters_per_unit": 1.0
      },
      "items": [
        {
          "item_id": "crate",
          "label": "Crate",
          "physics": {
            "mass_kg": 4.0,
            "collision": {
              "shape": "box",
              "half_extents_m": [0.25, 0.25, 0.25]
            }
          },
          "visual_parts": [
            {
              "part_id": "body",
              "file_id": "crate_mesh",
              "rgba": [0.7, 0.5, 0.3, 1.0],
              "scale": [1.0, 1.0, 1.0],
              "translation_m": [0.0, 0.0, 0.0],
              "rotation_wxyz": [1.0, 0.0, 0.0, 0.0]
            }
          ]
        }
      ]
    },
    "files": [
      {
        "file_id": "crate_mesh",
        "source_path": "models/crate.stl",
        "target_path": "meshes/crate.stl",
        "role": "visual_mesh",
        "media_type": "model/stl",
        "format": "stl"
      },
      {
        "file_id": "license_text",
        "source_path": "LICENSE.txt",
        "target_path": "evidence/LICENSE.txt",
        "role": "license",
        "media_type": "text/plain",
        "format": "text"
      },
      {
        "file_id": "source_readme",
        "source_path": "README.md",
        "target_path": "evidence/README.md",
        "role": "provenance",
        "media_type": "text/markdown",
        "format": "markdown"
      }
    ]
  }
}
```

导入命令：

```bash
python scripts/matrix_item_asset_import.py import \
  config/items/import-recipes/example-props.json \
  --source-root /home/trna/matrix-artifacts/example-props \
  --registry-root /home/trna/matrix-artifacts/item-packs
```

命令读取每个 source file 一次并在 staging 中同步计算 size/SHA256，验证生成的
canonical pack 后原子 rename 到
`sha256/<digest前两位>/<digest>/matrix-item-asset-pack.json`。同一 recipe 和
source bytes 重跑返回 `idempotent=true`；若同一 digest 位置已有不一致或损坏
内容则 fail closed。符号链接、非普通文件、路径逃逸、duplicate key、未知字段和
`NaN`/`Infinity` 都会被拒绝。

stdout 的 `digest_ref` 可直接写进独立的 `matrix-item-inventory/v1`。Importer
不会替调用方决定 slot、pool 或 spawn，所以不同机器/benchmark 可以复用同一个
pack，同时各自维护运行时 inventory 配置。

只读校验命令：

```bash
python scripts/matrix_item_asset_pack.py verify-pack /path/to/matrix-item-asset-pack.json
python scripts/matrix_item_asset_pack.py resolve-pack /path/to/registry sha256:<digest>
python scripts/matrix_item_asset_pack.py resolve-inventory inventory.json /path/to/registry
```
