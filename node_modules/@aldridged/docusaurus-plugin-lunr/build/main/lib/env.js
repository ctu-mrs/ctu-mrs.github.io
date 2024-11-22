"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const fs_extra_1 = __importDefault(require("fs-extra"));
const path_1 = __importDefault(require("path"));
const fp_1 = require("lodash/fp");
const constants_1 = require("./constants");
const VERSION_OPTIONS_DISABLED = {
    versioning: {
        docsDir: '',
        enabled: false,
        latestVersion: null,
        versions: [],
    }
};
function getVersionedDocsDir(siteDir) {
    return path_1.default.join(siteDir, constants_1.VERSIONED_DOCS_DIR);
}
function getVersionsJSONFile(siteDir) {
    return path_1.default.join(siteDir, constants_1.VERSIONS_JSON_FILE);
}
function getVersions(siteDir) {
    const versionsJSONFile = getVersionsJSONFile(siteDir);
    return fs_extra_1.default.existsSync(versionsJSONFile) ? JSON.parse(fs_extra_1.default.readFileSync(versionsJSONFile, 'utf8')) : [];
}
function getOptions(siteDir, versions) {
    return {
        versioning: {
            docsDir: getVersionedDocsDir(siteDir),
            enabled: true,
            latestVersion: fp_1.head(versions),
            versions,
        }
    };
}
function default_1(siteDir) {
    const versions = getVersions(siteDir);
    return fp_1.size(versions) ? getOptions(siteDir, versions) : VERSION_OPTIONS_DISABLED;
}
exports.default = default_1;
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiZW52LmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXMiOlsiLi4vLi4vLi4vc3JjL2xpYi9lbnYudHMiXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6Ijs7Ozs7QUFBQSx3REFBMEI7QUFDMUIsZ0RBQXdCO0FBRXhCLGtDQUF1QztBQUV2QywyQ0FBcUU7QUFHckUsTUFBTSx3QkFBd0IsR0FBRztJQUMvQixVQUFVLEVBQUU7UUFDVixPQUFPLEVBQUUsRUFBRTtRQUNYLE9BQU8sRUFBRSxLQUFLO1FBQ2QsYUFBYSxFQUFFLElBQUk7UUFDbkIsUUFBUSxFQUFFLEVBQUU7S0FDYjtDQUNGLENBQUM7QUFFRixTQUFTLG1CQUFtQixDQUFDLE9BQWU7SUFDMUMsT0FBTyxjQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sRUFBRSw4QkFBa0IsQ0FBQyxDQUFDO0FBQ2hELENBQUM7QUFFRCxTQUFTLG1CQUFtQixDQUFDLE9BQWU7SUFDMUMsT0FBTyxjQUFJLENBQUMsSUFBSSxDQUFDLE9BQU8sRUFBRSw4QkFBa0IsQ0FBQyxDQUFDO0FBQ2hELENBQUM7QUFFRCxTQUFTLFdBQVcsQ0FBQyxPQUFlO0lBQ2xDLE1BQU0sZ0JBQWdCLEdBQUcsbUJBQW1CLENBQUMsT0FBTyxDQUFDLENBQUM7SUFDdEQsT0FBTyxrQkFBRSxDQUFDLFVBQVUsQ0FBQyxnQkFBZ0IsQ0FBQyxDQUFDLENBQUMsQ0FBQyxJQUFJLENBQUMsS0FBSyxDQUFDLGtCQUFFLENBQUMsWUFBWSxDQUFDLGdCQUFnQixFQUFFLE1BQU0sQ0FBQyxDQUFDLENBQUMsQ0FBQyxDQUFDLEVBQUUsQ0FBQztBQUN0RyxDQUFDO0FBRUQsU0FBUyxVQUFVLENBQUMsT0FBZSxFQUFFLFFBQStCO0lBQ2xFLE9BQU87UUFDTCxVQUFVLEVBQUU7WUFDVixPQUFPLEVBQUUsbUJBQW1CLENBQUMsT0FBTyxDQUFDO1lBQ3JDLE9BQU8sRUFBRSxJQUFJO1lBQ2IsYUFBYSxFQUFFLFNBQUksQ0FBQyxRQUFRLENBQUM7WUFDN0IsUUFBUTtTQUNUO0tBQ0YsQ0FBQztBQUNKLENBQUM7QUFFRCxtQkFBeUIsT0FBZTtJQUN0QyxNQUFNLFFBQVEsR0FBRyxXQUFXLENBQUMsT0FBTyxDQUFDLENBQUM7SUFDdEMsT0FBTyxTQUFJLENBQUMsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDLFVBQVUsQ0FBQyxPQUFPLEVBQUUsUUFBUSxDQUFDLENBQUMsQ0FBQyxDQUFDLHdCQUF3QixDQUFDO0FBQ25GLENBQUM7QUFIRCw0QkFHQyJ9