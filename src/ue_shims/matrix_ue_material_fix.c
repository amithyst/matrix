#define _GNU_SOURCE

#include <errno.h>
#include <limits.h>
#include <link.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <uchar.h>
#include <unistd.h>

/*
 * Matrix 0.1.2 builds runtime static meshes without a material slot.  Its
 * SetMeshColor implementation consequently returns before applying color.
 *
 * This narrowly-scoped preload is a reversible bridge for the exact audited
 * Matrix UE executable below.  It supplies the cooked Engine basic-shape
 * material before runtime mesh build and redirects SetMeshColor from the
 * material's nonexistent "BaseColor" parameter to its "Color" parameter.
 * Every address and instruction sequence is guarded so an unknown binary
 * fails closed instead of receiving a best-effort patch.
 */

typedef struct MatrixLinearColor {
    float red;
    float green;
    float blue;
    float alpha;
} MatrixLinearColor;

typedef void *(*GetStaticMeshFn)(void **result, void *component);
typedef uint64_t (*AddMaterialFn)(void *static_mesh, void *material);
typedef void *(*GetRenderDataFn)(void *static_mesh);
typedef void *(*GetPrivateStaticClassFn)(void);
typedef void *(*StaticLoadObjectFn)(
    void *object_class,
    void *outer,
    const char16_t *name,
    const char16_t *filename,
    uint32_t load_flags,
    void *sandbox,
    _Bool allow_object_reconciliation,
    const void *instancing_context
);
typedef void (*SetMeshColorFn)(
    void *renderer,
    void *component,
    MatrixLinearColor color
);

enum {
    HOOK_PROLOGUE_SIZE = 12,
    TRAMPOLINE_SIZE = 4 + HOOK_PROLOGUE_SIZE + 5,
    MAX_G1_PROFILE_COLORS = 16,
    MAX_G1_PALETTE_LENGTH = 1024,
    MAX_G1_SKIN_ID_LENGTH = 48,
};

static const uintptr_t SET_MESH_COLOR_ADDRESS = UINT64_C(0x1077c580);
static const uintptr_t STATIC_MESH_COMPONENT_GET_STATIC_MESH_ADDRESS =
    UINT64_C(0x0b5c2230);
static const uintptr_t STATIC_MESH_ADD_MATERIAL_ADDRESS = UINT64_C(0x0ccd6a30);
static const uintptr_t STATIC_MESH_GET_RENDER_DATA_ADDRESS =
    UINT64_C(0x0cce9c50);
static const uintptr_t GENERATE_MESHES_ADD_MATERIAL_CALL_ADDRESS =
    UINT64_C(0x1077c3cb);
static const uintptr_t GENERATE_MESHES_ADD_MATERIAL_RETURN_ADDRESS =
    UINT64_C(0x1077c3d0);
static const uintptr_t MATERIAL_INTERFACE_STATIC_CLASS_ADDRESS =
    UINT64_C(0x0ba59cd0);
static const uintptr_t STATIC_LOAD_OBJECT_ADDRESS = UINT64_C(0x069f7710);
static const uintptr_t COLOR_NAME_IMMEDIATE_ADDRESS = UINT64_C(0x1077c5ee);
static const uint32_t EXPECTED_BASE_COLOR_NAME_ADDRESS = UINT32_C(0x0151c959);
static const uint32_t COLOR_NAME_ADDRESS = UINT32_C(0x015b0a2c);
static const char16_t BASIC_SHAPE_MATERIAL_PATH[] =
    u"/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial";
static const unsigned char EXPECTED_BUILD_ID[] = {
    0x05, 0x6e, 0x17, 0xb8, 0x67, 0x5b, 0x10, 0x06,
};

static const unsigned char EXPECTED_SET_MESH_COLOR_PROLOGUE[
    HOOK_PROLOGUE_SIZE
] = {
    0x41, 0x56,                         /* push r14 */
    0x53,                               /* push rbx */
    0x48, 0x83, 0xec, 0x38,             /* sub rsp, 0x38 */
    0x0f, 0x29, 0x4c, 0x24, 0x20,       /* movaps xmm1, [rsp + 0x20] */
};

static const unsigned char EXPECTED_ADD_MATERIAL_PROLOGUE[
    HOOK_PROLOGUE_SIZE
] = {
    0x41, 0x57,                         /* push r15 */
    0x41, 0x56,                         /* push r14 */
    0x41, 0x55,                         /* push r13 */
    0x41, 0x54,                         /* push r12 */
    0x53,                               /* push rbx */
    0x48, 0x85, 0xf6,                   /* test rsi, rsi */
};

static const unsigned char EXPECTED_ADD_MATERIAL_CALL[] = {
    0xe8, 0x60, 0xa6, 0x55, 0xfc,
};

static SetMeshColorFn original_set_mesh_color;
static AddMaterialFn original_add_material;
static unsigned int substituted_material_count;
static unsigned int repaired_section_count;
static float g1_profile_colors[MAX_G1_PROFILE_COLORS][3];
static size_t g1_profile_color_count;
static char g1_skin_id[MAX_G1_SKIN_ID_LENGTH + 1];
static float g1_scope_alpha;

typedef struct MainImageRange {
    uintptr_t address;
    size_t size;
    uint32_t required_flags;
    int found;
} MainImageRange;

static void write_message(const char *message)
{
    size_t length = strlen(message);
    while (length > 0) {
        ssize_t written = write(STDERR_FILENO, message, length);
        if (written > 0) {
            message += written;
            length -= (size_t)written;
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        break;
    }
}

static void fail_closed(const char *reason)
{
    char message[512];
    int length = snprintf(
        message,
        sizeof(message),
        "matrix-ue-material-fix FATAL: %s\n",
        reason
    );
    if (length > 0) {
        write_message(message);
    }
    _exit(86);
}

static int is_valid_skin_id(const char *value)
{
    size_t length = value != NULL ? strlen(value) : 0U;
    if (length == 0U || length > MAX_G1_SKIN_ID_LENGTH) {
        return 0;
    }
    for (size_t index = 0; index < length; ++index) {
        unsigned char character = (unsigned char)value[index];
        if ((character < 'a' || character > 'z')
            && (character < '0' || character > '9')
            && (index == 0U || character != '-')) {
            return 0;
        }
    }
    return 1;
}

static float parse_palette_component(const char **cursor)
{
    if ((**cursor < '0' || **cursor > '9') && **cursor != '.') {
        fail_closed("G1 material palette contains an invalid component");
    }
    errno = 0;
    char *end = NULL;
    float value = strtof(*cursor, &end);
    if (end == *cursor || errno == ERANGE || !isfinite(value)
        || value < 0.0f || value > 1.0f) {
        fail_closed("G1 material palette component is outside [0, 1]");
    }
    *cursor = end;
    return value;
}

static void load_g1_material_palette(void)
{
    const char *configured_skin = getenv("MATRIX_G1_SKIN");
    const char *palette = getenv("MATRIX_G1_MATERIAL_PALETTE");
    const char *scope_alpha = getenv("MATRIX_G1_MATERIAL_SCOPE_ALPHA");
    if (!is_valid_skin_id(configured_skin)) {
        fail_closed("MATRIX_G1_SKIN is missing or invalid");
    }
    if (palette == NULL || palette[0] == '\0'
        || strlen(palette) > MAX_G1_PALETTE_LENGTH) {
        fail_closed("MATRIX_G1_MATERIAL_PALETTE is missing or too long");
    }
    memcpy(g1_skin_id, configured_skin, strlen(configured_skin) + 1U);
    if (scope_alpha == NULL || scope_alpha[0] == '\0') {
        fail_closed("MATRIX_G1_MATERIAL_SCOPE_ALPHA is missing");
    }
    const char *scope_cursor = scope_alpha;
    g1_scope_alpha = parse_palette_component(&scope_cursor);
    if (*scope_cursor != '\0' || g1_scope_alpha <= 0.0f
        || g1_scope_alpha > 0.999f) {
        fail_closed("G1 material scope alpha must be inside (0, 0.999]");
    }

    const char *cursor = palette;
    while (*cursor != '\0') {
        if (g1_profile_color_count >= MAX_G1_PROFILE_COLORS) {
            fail_closed("G1 material palette contains too many colors");
        }
        for (size_t channel = 0; channel < 3U; ++channel) {
            g1_profile_colors[g1_profile_color_count][channel] =
                parse_palette_component(&cursor);
            if (channel < 2U) {
                if (*cursor != ',') {
                    fail_closed("G1 material palette must use r,g,b triples");
                }
                ++cursor;
            }
        }
        ++g1_profile_color_count;
        if (*cursor == '\0') {
            break;
        }
        if (*cursor != ';' || cursor[1] == '\0') {
            fail_closed("G1 material palette must separate colors with ';'");
        }
        ++cursor;
    }
    if (g1_profile_color_count == 0U) {
        fail_closed("G1 material palette contains no colors");
    }
    if (unsetenv("MATRIX_G1_MATERIAL_PALETTE") != 0
        || unsetenv("MATRIX_G1_MATERIAL_SCOPE_ALPHA") != 0
        || unsetenv("MATRIX_G1_SKIN") != 0) {
        fail_closed("could not clear the selected G1 skin environment");
    }

    char message[192];
    int length = snprintf(
        message,
        sizeof(message),
        "matrix-ue-material-fix: loaded skin %s palette (%zu colors)\n",
        g1_skin_id,
        g1_profile_color_count
    );
    if (length > 0) {
        write_message(message);
    }
}

static int find_main_image_range(
    struct dl_phdr_info *info,
    size_t info_size,
    void *opaque
)
{
    (void)info_size;
    MainImageRange *range = (MainImageRange *)opaque;
    if (info->dlpi_name != NULL && info->dlpi_name[0] != '\0') {
        return 0;
    }
    if (range->size > UINTPTR_MAX - range->address) {
        return 1;
    }
    uintptr_t requested_end = range->address + range->size;
    for (ElfW(Half) index = 0; index < info->dlpi_phnum; ++index) {
        const ElfW(Phdr) *header = &info->dlpi_phdr[index];
        if (header->p_type != PT_LOAD) {
            continue;
        }
        uintptr_t start = (uintptr_t)info->dlpi_addr + header->p_vaddr;
        if (header->p_memsz > UINTPTR_MAX - start) {
            continue;
        }
        uintptr_t end = start + header->p_memsz;
        if (range->address >= start && requested_end <= end
            && (header->p_flags & range->required_flags)
                == range->required_flags) {
            range->found = 1;
            return 1;
        }
    }
    return 1;
}

static int main_image_contains(
    uintptr_t address,
    size_t size,
    uint32_t required_flags
)
{
    MainImageRange range = {
        .address = address,
        .size = size,
        .required_flags = required_flags,
        .found = 0,
    };
    dl_iterate_phdr(find_main_image_range, &range);
    return range.found;
}

typedef struct MainBuildId {
    int matches;
} MainBuildId;

static size_t align_note_size(size_t size)
{
    return (size + 3U) & ~((size_t)3U);
}

static int find_main_build_id(
    struct dl_phdr_info *info,
    size_t info_size,
    void *opaque
)
{
    (void)info_size;
    MainBuildId *result = (MainBuildId *)opaque;
    if (info->dlpi_name != NULL && info->dlpi_name[0] != '\0') {
        return 0;
    }
    for (ElfW(Half) index = 0; index < info->dlpi_phnum; ++index) {
        const ElfW(Phdr) *header = &info->dlpi_phdr[index];
        if (header->p_type != PT_NOTE) {
            continue;
        }
        const unsigned char *cursor = (const unsigned char *)(
            (uintptr_t)info->dlpi_addr + header->p_vaddr
        );
        size_t remaining = (size_t)header->p_memsz;
        while (remaining >= sizeof(ElfW(Nhdr))) {
            ElfW(Nhdr) note;
            memcpy(&note, cursor, sizeof(note));
            cursor += sizeof(note);
            remaining -= sizeof(note);
            size_t name_size = align_note_size(note.n_namesz);
            size_t description_size = align_note_size(note.n_descsz);
            if (name_size > remaining
                || description_size > remaining - name_size) {
                break;
            }
            const unsigned char *name = cursor;
            const unsigned char *description = cursor + name_size;
            if (note.n_type == NT_GNU_BUILD_ID
                && note.n_namesz >= 3
                && memcmp(name, "GNU", 3) == 0
                && note.n_descsz == sizeof(EXPECTED_BUILD_ID)
                && memcmp(
                    description,
                    EXPECTED_BUILD_ID,
                    sizeof(EXPECTED_BUILD_ID)
                ) == 0) {
                result->matches = 1;
                return 1;
            }
            cursor += name_size + description_size;
            remaining -= name_size + description_size;
        }
    }
    return 1;
}

static int main_image_has_expected_build_id(void)
{
    MainBuildId result = {.matches = 0};
    dl_iterate_phdr(find_main_build_id, &result);
    return result.matches;
}

static void write_absolute_jump(unsigned char *destination, const void *target)
{
    uintptr_t address = (uintptr_t)target;
    destination[0] = 0x48;
    destination[1] = 0xb8; /* movabs rax, imm64 */
    memcpy(destination + 2, &address, sizeof(address));
    destination[10] = 0xff;
    destination[11] = 0xe0; /* jmp rax */
}

static void write_relative_jump(unsigned char *destination, const void *target)
{
    int64_t displacement = (int64_t)(uintptr_t)target
        - (int64_t)(uintptr_t)(destination + 5);
    if (displacement < INT32_MIN || displacement > INT32_MAX) {
        fail_closed("hook trampoline is outside rel32 range");
    }
    int32_t encoded = (int32_t)displacement;
    destination[0] = 0xe9;
    memcpy(destination + 1, &encoded, sizeof(encoded));
}

static void *load_basic_shape_material(void)
{
    GetPrivateStaticClassFn get_material_interface_class =
        (GetPrivateStaticClassFn)MATERIAL_INTERFACE_STATIC_CLASS_ADDRESS;
    void *material_interface_class = get_material_interface_class();
    if (material_interface_class == NULL) {
        fail_closed("UMaterialInterface class is unavailable at mesh creation");
    }
    StaticLoadObjectFn static_load_object =
        (StaticLoadObjectFn)STATIC_LOAD_OBJECT_ADDRESS;
    void *material = static_load_object(
        material_interface_class,
        NULL,
        BASIC_SHAPE_MATERIAL_PATH,
        NULL,
        0U,
        NULL,
        1,
        NULL
    );
    if (material == NULL) {
        fail_closed("could not load cooked BasicShapeMaterial");
    }
    return material;
}

static uint64_t matrix_add_material_hook(
    void *static_mesh,
    void *material
)
{
    uintptr_t caller = (uintptr_t)__builtin_return_address(0);
    if (material == NULL
        && caller == GENERATE_MESHES_ADD_MATERIAL_RETURN_ADDRESS) {
        material = load_basic_shape_material();
        if (__atomic_fetch_add(
                &substituted_material_count,
                1U,
                __ATOMIC_RELAXED
            ) == 0U) {
            write_message(
                "matrix-ue-material-fix: supplied BasicShapeMaterial before "
                "runtime static mesh build\n"
            );
        }
    }
    return original_add_material(static_mesh, material);
}

static int component_matches(float actual, float expected)
{
    float difference = actual - expected;
    return difference > -0.0001f && difference < 0.0001f;
}

static int is_g1_material_profile_color(MatrixLinearColor color)
{
    for (size_t index = 0; index < g1_profile_color_count; ++index) {
        if (component_matches(color.red, g1_profile_colors[index][0])
            && component_matches(color.green, g1_profile_colors[index][1])
            && component_matches(color.blue, g1_profile_colors[index][2])
            && component_matches(color.alpha, g1_scope_alpha)) {
            return 1;
        }
    }
    return 0;
}

static void repair_first_runtime_section_material(void *component)
{
    GetStaticMeshFn get_static_mesh =
        (GetStaticMeshFn)STATIC_MESH_COMPONENT_GET_STATIC_MESH_ADDRESS;
    void *static_mesh = NULL;
    get_static_mesh(&static_mesh, component);
    if (static_mesh == NULL) {
        return;
    }

    GetRenderDataFn get_render_data =
        (GetRenderDataFn)STATIC_MESH_GET_RENDER_DATA_ADDRESS;
    unsigned char *render_data = get_render_data(static_mesh);
    if (render_data == NULL) {
        return;
    }

    /*
     * Audited UE 5.5 layout for this Build ID:
     * FStaticMeshRenderData starts with a TIndirectArray of LOD pointers;
     * LOD0 has an inline one-element Sections array at +0x10, with ArrayNum
     * at +0x38.  FStaticMeshSection::MaterialIndex is its first int32.
     */
    void **lod_resources = NULL;
    int lod_count = 0;
    memcpy(&lod_resources, render_data, sizeof(lod_resources));
    memcpy(&lod_count, render_data + 8, sizeof(lod_count));
    if (lod_resources == NULL || lod_count <= 0 || lod_count > 8
        || lod_resources[0] == NULL) {
        return;
    }

    unsigned char *lod0 = (unsigned char *)lod_resources[0];
    int section_count = 0;
    memcpy(&section_count, lod0 + 0x38, sizeof(section_count));
    if (section_count != 1) {
        return;
    }

    int material_index = 0;
    memcpy(&material_index, lod0 + 0x10, sizeof(material_index));
    if (material_index == -1) {
        int slot_zero = 0;
        memcpy(lod0 + 0x10, &slot_zero, sizeof(slot_zero));
        if (__atomic_fetch_add(
                &repaired_section_count,
                1U,
                __ATOMIC_RELAXED
            ) == 0U) {
            write_message(
                "matrix-ue-material-fix: mapped G1 material profile "
                "section to slot 0\n"
            );
        }
    }
}

static void matrix_set_mesh_color_hook(
    void *renderer,
    void *component,
    MatrixLinearColor color
)
{
    if (component != NULL && is_g1_material_profile_color(color)) {
        repair_first_runtime_section_material(component);
        color.alpha = 1.0f;
    }

    original_set_mesh_color(renderer, component, color);
}

__attribute__((constructor)) static void install_matrix_material_fix(void)
{
    unsigned char *set_mesh_color =
        (unsigned char *)SET_MESH_COLOR_ADDRESS;
    unsigned char *add_material =
        (unsigned char *)STATIC_MESH_ADD_MATERIAL_ADDRESS;
    uint32_t *color_name_immediate =
        (uint32_t *)COLOR_NAME_IMMEDIATE_ADDRESS;

    if (!main_image_has_expected_build_id()) {
        fail_closed("main executable Build ID is not Matrix 0.1.2");
    }
    load_g1_material_palette();
    if (!main_image_contains(
            SET_MESH_COLOR_ADDRESS,
            0x80,
            PF_R | PF_X
        )
        || !main_image_contains(
            EXPECTED_BASE_COLOR_NAME_ADDRESS,
            sizeof("BaseColor"),
            PF_R
        )
        || !main_image_contains(COLOR_NAME_ADDRESS, sizeof("Color"), PF_R)
        || !main_image_contains(
            STATIC_MESH_COMPONENT_GET_STATIC_MESH_ADDRESS,
            1,
            PF_R | PF_X
        )
        || !main_image_contains(
            STATIC_MESH_ADD_MATERIAL_ADDRESS,
            1,
            PF_R | PF_X
        )
        || !main_image_contains(
            STATIC_MESH_GET_RENDER_DATA_ADDRESS,
            1,
            PF_R | PF_X
        )
        || !main_image_contains(
            GENERATE_MESHES_ADD_MATERIAL_CALL_ADDRESS,
            sizeof(EXPECTED_ADD_MATERIAL_CALL),
            PF_R | PF_X
        )
        || !main_image_contains(
            MATERIAL_INTERFACE_STATIC_CLASS_ADDRESS,
            1,
            PF_R | PF_X
        )
        || !main_image_contains(
            STATIC_LOAD_OBJECT_ADDRESS,
            1,
            PF_R | PF_X
        )) {
        fail_closed("audited Matrix 0.1.2 image ranges are unavailable");
    }

    if (memcmp(
            set_mesh_color,
            EXPECTED_SET_MESH_COLOR_PROLOGUE,
            HOOK_PROLOGUE_SIZE
        ) != 0) {
        fail_closed("SetMeshColor prologue does not match Matrix 0.1.2");
    }
    if (memcmp(
            add_material,
            EXPECTED_ADD_MATERIAL_PROLOGUE,
            HOOK_PROLOGUE_SIZE
        ) != 0) {
        fail_closed("UStaticMesh::AddMaterial prologue does not match");
    }
    if (memcmp(
            (const void *)GENERATE_MESHES_ADD_MATERIAL_CALL_ADDRESS,
            EXPECTED_ADD_MATERIAL_CALL,
            sizeof(EXPECTED_ADD_MATERIAL_CALL)
        ) != 0) {
        fail_closed("GenerateMeshes AddMaterial callsite does not match");
    }
    uint32_t current_color_name_address;
    memcpy(
        &current_color_name_address,
        color_name_immediate,
        sizeof(current_color_name_address)
    );
    if (current_color_name_address != EXPECTED_BASE_COLOR_NAME_ADDRESS) {
        fail_closed("SetMeshColor BaseColor literal does not match Matrix 0.1.2");
    }
    if (strcmp(
            (const char *)(uintptr_t)EXPECTED_BASE_COLOR_NAME_ADDRESS,
            "BaseColor"
        ) != 0) {
        fail_closed("audited BaseColor FName literal is unavailable");
    }
    if (strcmp((const char *)(uintptr_t)COLOR_NAME_ADDRESS, "Color") != 0) {
        fail_closed("audited Color FName literal is unavailable");
    }

    unsigned char *trampoline = mmap(
        NULL,
        TRAMPOLINE_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT,
        -1,
        0
    );
    if (trampoline == MAP_FAILED) {
        fail_closed("could not allocate SetMeshColor trampoline");
    }

    /* The hook calls this entry indirectly, so retain an ENDBR landing pad. */
    trampoline[0] = 0xf3;
    trampoline[1] = 0x0f;
    trampoline[2] = 0x1e;
    trampoline[3] = 0xfa; /* endbr64 */
    memcpy(
        trampoline + 4,
        EXPECTED_SET_MESH_COLOR_PROLOGUE,
        HOOK_PROLOGUE_SIZE
    );
    write_relative_jump(
        trampoline + 4 + HOOK_PROLOGUE_SIZE,
        set_mesh_color + HOOK_PROLOGUE_SIZE
    );
    if (mprotect(trampoline, TRAMPOLINE_SIZE, PROT_READ | PROT_EXEC) != 0) {
        fail_closed("could not seal SetMeshColor trampoline executable");
    }
    original_set_mesh_color = (SetMeshColorFn)trampoline;

    unsigned char *add_material_trampoline = mmap(
        NULL,
        TRAMPOLINE_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS | MAP_32BIT,
        -1,
        0
    );
    if (add_material_trampoline == MAP_FAILED) {
        fail_closed("could not allocate AddMaterial trampoline");
    }
    add_material_trampoline[0] = 0xf3;
    add_material_trampoline[1] = 0x0f;
    add_material_trampoline[2] = 0x1e;
    add_material_trampoline[3] = 0xfa; /* endbr64 */
    memcpy(
        add_material_trampoline + 4,
        EXPECTED_ADD_MATERIAL_PROLOGUE,
        HOOK_PROLOGUE_SIZE
    );
    write_relative_jump(
        add_material_trampoline + 4 + HOOK_PROLOGUE_SIZE,
        add_material + HOOK_PROLOGUE_SIZE
    );
    if (mprotect(
            add_material_trampoline,
            TRAMPOLINE_SIZE,
            PROT_READ | PROT_EXEC
        ) != 0) {
        fail_closed("could not seal AddMaterial trampoline executable");
    }
    original_add_material = (AddMaterialFn)add_material_trampoline;

    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        fail_closed("could not determine memory page size");
    }
    uintptr_t page = SET_MESH_COLOR_ADDRESS & ~((uintptr_t)page_size - 1U);
    uintptr_t add_material_page = STATIC_MESH_ADD_MATERIAL_ADDRESS
        & ~((uintptr_t)page_size - 1U);
    if (mprotect(
            (void *)page,
            (size_t)page_size,
            PROT_READ | PROT_WRITE
        ) != 0) {
        fail_closed("could not make SetMeshColor writable");
    }
    if (mprotect(
            (void *)add_material_page,
            (size_t)page_size,
            PROT_READ | PROT_WRITE
        ) != 0) {
        fail_closed("could not make AddMaterial writable");
    }

    uint32_t corrected_color_name_address = COLOR_NAME_ADDRESS;
    memcpy(
        color_name_immediate,
        &corrected_color_name_address,
        sizeof(corrected_color_name_address)
    );
    unsigned char hook_jump[HOOK_PROLOGUE_SIZE];
    write_absolute_jump(hook_jump, (const void *)matrix_set_mesh_color_hook);
    memcpy(set_mesh_color, hook_jump, sizeof(hook_jump));
    __builtin___clear_cache(
        (char *)set_mesh_color,
        (char *)set_mesh_color + HOOK_PROLOGUE_SIZE
    );
    write_absolute_jump(
        hook_jump,
        (const void *)matrix_add_material_hook
    );
    memcpy(add_material, hook_jump, sizeof(hook_jump));
    __builtin___clear_cache(
        (char *)add_material,
        (char *)add_material + HOOK_PROLOGUE_SIZE
    );

    if (mprotect((void *)page, (size_t)page_size, PROT_READ | PROT_EXEC) != 0) {
        fail_closed("could not restore SetMeshColor page protections");
    }
    if (mprotect(
            (void *)add_material_page,
            (size_t)page_size,
            PROT_READ | PROT_EXEC
        ) != 0) {
        fail_closed("could not restore AddMaterial page protections");
    }
    if (unsetenv("LD_PRELOAD") != 0) {
        fail_closed("could not clear inherited LD_PRELOAD after installation");
    }
    write_message(
        "matrix-ue-material-fix: installed audited Matrix 0.1.2 material bridge\n"
    );
}
